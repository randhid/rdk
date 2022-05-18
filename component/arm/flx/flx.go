// Package flx implements a flxbot arm
package flx

import (
	"context"
	exec "os/exec"

	// for embedding model kinematics file.
	_ "embed"
	"fmt"
	"strconv"
	"sync"
	"time"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"

	"go.viam.com/rdk/component/arm"
	"go.viam.com/rdk/component/generic"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/motionplan"
	commonpb "go.viam.com/rdk/proto/api/common/v1"
	pb "go.viam.com/rdk/proto/api/component/arm/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/registry"
	"go.viam.com/rdk/robot"
)

const (
	modelname = "flx"
)

// AttrConfig is used for converting attributes.
type AttrConfig struct {
	Host      string `json:"host"`
	NumFlxSeg int    `json:"number_of_flx_segments"`
	Mode      string `json:"flxbot_mode"`
}

//go:embed flxbot_base_SVA.json
var flxbotBasejson []byte

//go:embed flxbot_segment_SVA.json
var flxbotSegmentjson []byte

//go:embed flxbot_end_effector_SVA.json
var flxbotEndEffectorjson []byte

//go:embed flxbot_fixed_segment_SVA.json
var flxbotFixedjson []byte

type flxbotmode string

const (
	flxfoab = flxbotmode("foab")
	flxseg  = flxbotmode("segments")
)

type flxArm struct {
	generic.Unimplemented
	host     string
	model    referenceframe.Model
	mode     flxbotmode
	numSeg   int
	mp       motionplan.MotionPlanner
	moveLock *sync.Mutex
	logger   golog.Logger
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
		host:          flxconf.Host,
		mode:          flxbotmode(flxconf.Mode),
		numSeg:        flxconf.NumFlxSeg,
		moveLock:      &sync.Mutex{},
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
		}
	}

	mp, err := motionplan.NewCBiRRTMotionPlanner(newArm.model, 4, logger)
	if err != nil {
		return nil, err
	}
	newArm.mp = mp

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
	joints, err := flx.GetJointPositions(ctx)
	if err != nil {
		return nil, err
	}
	return motionplan.ComputePosition(flx.mp.Frame(), joints)
}

func (flx *flxArm) MoveToPosition(ctx context.Context, pos *commonpb.Pose, worldState *commonpb.WorldState) error {
	joints, err := flx.GetJointPositions(ctx)
	if err != nil {
		return err
	}

	start := time.Now()
	solution, err := flx.mp.Plan(ctx, pos, referenceframe.JointPosToInputs(joints), nil)
	if err != nil {
		return err
	}

	if time.Since(start) > 2*time.Second {
		return errors.Errorf("flx took too long to solve for new position %v", time.Since(start))
	}
	return arm.GoToWaypoints(ctx, flx, solution)
}

func (flx *flxArm) MoveToJointPositions(ctx context.Context, newPositions *pb.JointPositions) error {
	positions := newPositions.GetDegrees()
	positionsStr := ""
	for _, pos := range positions {
		positionsStr += fmt.Sprintf("%f ", pos)
	}
	cmd := exec.Command("python", "flxbot_move_to_joint_position.py", positionsStr, "10", "10")
	return cmd.Run()
}

func (flx *flxArm) GetJointPositions(ctx context.Context) (*pb.JointPositions, error) {
	// cmd := exec.Command("python", "flxbot_python/flxbot_state.py")
	// err := cmd.Run()

	// out, err := cmd.CombinedOutput()

	// if err != nil {
	// return nil, err
	// }
	// out := string("0 0 0 0 0 0 0")
	// posStrings := strings.Split(string(out), " ")

	// result := &pb.JointPositions{}
	// degs := make([]float64, 0, len(posStrings))
	// for i, posStr := range posStrings {
	// pos, err := strconv.ParseFloat(posStr, 64)
	// if err != nil {
	// return nil, err
	// }
	// degs[i] = pos
	// }
	// result.Degrees = degs
	// return result, nil
	degs := make([]float64, len(flx.model.DoF()))
	for i := 0; i < cap(degs); i++ {
		degs[i] = float64(i)
	}

	return &pb.JointPositions{Degrees: degs}, nil
}

func (flx *flxArm) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	res, err := flx.GetJointPositions(ctx)
	if err != nil {
		return nil, err
	}
	return referenceframe.JointPosToInputs(res), nil
}

func (flx *flxArm) GoToInputs(ctx context.Context, goal []referenceframe.Input) error {
	return flx.MoveToJointPositions(ctx, referenceframe.InputsToJointPos(goal))
}
