// Package flx implements a flxbot arm
package flx

import (
	"context"
	"math"
	exec "os/exec"
	"strings"

	// for embedding model kinematics file.
	_ "embed"
	"fmt"
	"strconv"
	"sync"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"

	"go.viam.com/rdk/component/arm"
	"go.viam.com/rdk/component/generic"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/operation"
	commonpb "go.viam.com/rdk/proto/api/common/v1"
	pb "go.viam.com/rdk/proto/api/component/arm/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/registry"
	"go.viam.com/rdk/robot"
)

// AttrConfig is used for converting attributes.
type AttrConfig struct {
	NumFlxSeg int    `json:"number_of_flx_segments"`
	Mode      string `json:"flxbot_mode"`
	InPlane   bool   `json:"move_in_plane"`
}

const (
	modelname = "flx"
)

//go:embed flxbot_segment_SVA.json
var flxbotSegmentjson []byte

//go:embed flxbot_fixed_segment_SVA.json
var flxbotFixedjson []byte

type flxbotmode string

const (
	flxfoab = flxbotmode("foab")
	flxseg  = flxbotmode("segments")
)

type flxArm struct {
	generic.Unimplemented
	host    string
	model   referenceframe.Model
	mode    flxbotmode
	numSeg  int
	mp      motionplan.MotionPlanner
	mu      *sync.Mutex
	logger  golog.Logger
	moving  bool
	opMgr   operation.SingleOperationManager
	inPlane bool
}

func init() {
	registry.RegisterComponent(arm.Subtype, modelname, registry.Component{
		Constructor: func(
			ctx context.Context,
			r robot.Robot,
			config config.Component,
			logger golog.Logger) (interface{}, error) {
			return newFLX(ctx, config, logger)
		},
	})

	config.RegisterComponentAttributeMapConverter(arm.SubtypeName, modelname,
		func(attributes config.AttributeMap) (interface{}, error) {
			var conf AttrConfig
			return config.TransformAttributeMapToStruct(&conf, attributes)
		},
		&AttrConfig{})
}

func newFLX(ctx context.Context, cfg config.Component, logger golog.Logger) (arm.Arm, error) {
	flxconf, ok := cfg.ConvertedAttributes.(*AttrConfig)
	if !ok {
		return nil, errors.New("flxArm Attributes invalid")
	}

	newArm := &flxArm{
		Unimplemented: generic.Unimplemented{},
		mode:          flxbotmode(flxconf.Mode),
		numSeg:        flxconf.NumFlxSeg,
		inPlane:       flxconf.InPlane,
		mu:            &sync.Mutex{},
		logger:        logger,
	}

	switch newArm.mode {
	case flxfoab:
		model, err := foabModel()
		newArm.model = model
		if err != nil {
			return nil, err
		}
	case flxseg:
		model, err := flxArmModel(newArm.numSeg)
		newArm.model = model
		if err != nil {
			return nil, err
		} else {
			return nil, errors.New("flx segment modular is currently experimental")
		}
	}

	if newArm.inPlane {
		mp, err := motionplan.NewFLXMotionPlanner(newArm.model, 4, logger)
		if err != nil {
			return nil, err
		}
		newArm.mp = mp
	} else {
		mp, err := motionplan.NewCBiRRTMotionPlanner(newArm.model, 4, logger)
		if err != nil {
			return nil, err
		}
		newArm.mp = mp
	}

	return newArm, nil
}

func foabModel() (referenceframe.Model, error) {
	model := referenceframe.NewSimpleModel()

	fixm, err := referenceframe.UnmarshalModelJSON(flxbotFixedjson, "base")
	if err != nil {
		return nil, err
	}
	segm, err := referenceframe.UnmarshalModelJSON(flxbotSegmentjson, "segment")
	if err != nil {
		return nil, err
	}
	model.OrdTransforms = append(model.OrdTransforms, fixm, segm)
	model.ChangeName("foab")
	return model, nil
}

func flxArmModel(numSeg int) (referenceframe.Model, error) {
	model := referenceframe.NewSimpleModel()

	fixm, err := referenceframe.UnmarshalModelJSON(flxbotFixedjson, "fixed")
	if err != nil {
		return nil, err
	}
	model.OrdTransforms = append(model.OrdTransforms, fixm)

	for idx := 0; idx < numSeg; idx++ {
		segname := "seg" + strconv.Itoa(idx)
		fmt.Print(segname)
		seg, err := referenceframe.UnmarshalModelJSON(flxbotSegmentjson, segname)
		if err != nil {
			return nil, err
		}
		model.OrdTransforms = append(model.OrdTransforms, seg)
	}

	model.ChangeName("flxarm")
	return model, nil
}

func (flx *flxArm) ModelFrame() referenceframe.Model {
	return flx.model
}

func (flx *flxArm) GetEndPosition(ctx context.Context) (*commonpb.Pose, error) {
	flx.mu.Lock()
	defer flx.mu.Unlock()

	joints, err := flx.GetJointPositions(ctx)
	if err != nil {
		return nil, err
	}
	return motionplan.ComputePosition(flx.mp.Frame(), joints)
}

func (flx *flxArm) MoveToPosition(ctx context.Context, pos *commonpb.Pose, worldState *commonpb.WorldState) error {
	ctx, done := flx.opMgr.New(ctx)
	defer done()

	startJoints, err := flx.GetJointPositions(ctx)
	if err != nil {
		return err
	}

	// //iterate over in plane plan.
	// startPos, err := flx.GetEndPosition(ctx)
	// if err != nil {
	// 	return err
	// }

	var solution [][]referenceframe.Input
	if flx.inPlane {
		//iterate over in plane plan. put things in driver
		sol, err := flx.mp.Plan(ctx, pos, referenceframe.FloatsToInputs(startJoints.Degrees), nil)
		if err != nil {
			return err
		}
		solution = sol
	} else {
		// change to cbirrt with plane constraint
		opt := &motionplan.PlannerOptions{}

		sol, err := flx.mp.Plan(ctx, pos, referenceframe.JointPosToInputs(startJoints), opt)
		if err != nil {
			return err
		}
		solution = sol

	}

	return arm.GoToWaypoints(ctx, flx, solution)
}

func GetAngles(startAngle, endAngle, tarVel, tarAcc, dt float64) []float64 {
	currVel := 0.0
	angles := []float64{startAngle}

	var dir float64
	if endAngle > startAngle {
		dir = 1
	} else {
		dir = -1
	}
	for currVel < tarVel {
		idxEnd := len(angles) - 1
		angles = append(angles, angles[idxEnd]+dir*currVel*dt+dir*tarAcc*0.5*dt*dt)
		currVel = tarVel
		angleDiff := math.Abs(angles[idxEnd] - angles[0])
		for math.Abs(endAngle-angles[idxEnd]) > angleDiff {
			angles = append(angles, dir*tarVel*dt)
		}
		for currVel*dir > 0 {
			angles = append(angles, angles[idxEnd]+dir*currVel*dt+dir*tarAcc*0.5*dt*dt)
			currVel = math.Max(currVel-tarAcc*dt, 0.0)
		}
	}
	angles = append(angles, endAngle)
	return angles
}

func (flx *flxArm) GoToInputs(ctx context.Context, goal []referenceframe.Input) error {
	return flx.MoveToJointPositions(ctx, referenceframe.InputsToJointPos(goal))
}

func (flx *flxArm) MoveToJointPositions(ctx context.Context, newPositions *pb.JointPositions) error {
	ctx, done := flx.opMgr.New(ctx)
	defer done()

	flx.mu.Lock()
	defer flx.mu.Unlock()

	positions := newPositions.GetDegrees()
	if len(positions) != len(newPositions.Degrees) {
		return errors.New("joint position mismatch for flxbot")
	}
	positionsStr := ""

	for _, pos := range positions {
		positionsStr += fmt.Sprintf("%f ", pos)
	}

	cmd := exec.Command("python", "flxbot_move_to_joint_position.py", positionsStr, "10", "10")
	// cmd := exec.Command("python", "flxbot_move_to_joint_position.py", positionsStr, "10", "10")
	return cmd.Run()
}

func (flx *flxArm) GetJointPositions(ctx context.Context) (*pb.JointPositions, error) {
	cmd := exec.Command("python", "flxbot_python/flxbot_state.py")
	err := cmd.Run()

	out, err := cmd.CombinedOutput()
	fmt.Println(out)

	if err != nil {
		return nil, err
	}
	// out := string("0 0 0 0 0 0 0")
	posStrings := strings.Split(string(out), " ")

	result := &pb.JointPositions{}
	degs := make([]float64, 0, len(posStrings))
	for i, posStr := range posStrings {
		pos, err := strconv.ParseFloat(posStr, 64)
		if err != nil {
			return nil, err
		}
		degs[i] = pos
	}
	result.Degrees = degs
	return result, nil
	// degs := make([]float64, len(flx.model.DoF()))
	// for i := 0; i < cap(degs); i++ {
	// 	if i < 2 {
	// 		degs[i] = 0
	// 	} else {
	// 		degs[i] = float64(i)
	// 	}
	// }
	// return &pb.JointPositions{Degrees: degs}, nil
}

func (flx *flxArm) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	res, err := flx.GetJointPositions(ctx)
	if err != nil {
		return nil, err
	}
	return referenceframe.JointPosToInputs(res), nil
}

func (flx *flxArm) Stop(ctx context.Context) error {
	positions, err := flx.GetJointPositions(ctx)
	if err != nil {
		return err
	}

	positionsStr := ""
	for _, pos := range positions.Degrees {
		positionsStr += fmt.Sprintf("%f ", pos)
	}

	cmd := exec.Command("python", "flxbot_move_to_joint_position.py", positionsStr, "0", "0")
	return cmd.Run()

}

func (flx *flxArm) IsMoving() bool {
	return flx.opMgr.OpRunning()
}

// IMPLEMENT GETCOMPONENTS?
