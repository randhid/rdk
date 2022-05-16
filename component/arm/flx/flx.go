// Package flx implements a flxbot arm
package flx

import (
	"context"

	// for embedding model kinematics file.
	_ "embed"
	"sync"

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
			return newFLX(config, logger)
		},
	})

	config.RegisterComponentAttributeMapConverter(arm.SubtypeName, modelname,
		func(attributes config.AttributeMap) (interface{}, error) {
			var conf AttrConfig
			return config.TransformAttributeMapToStruct(&conf, attributes)
		},
		&AttrConfig{})
}

func newFLX(cfg config.Component, logger golog.Logger) (arm.Arm, error) {
	flxconf, ok := cfg.ConvertedAttributes.(*AttrConfig)
	if !ok {
		return nil, errors.New("flxArm Attributes invalid")
	}

	newArm := &flxArm{
		host:     flxconf.Host,
		mode:     flxbotmode(flxconf.Mode),
		numSeg:   flxconf.NumFlxSeg,
		moveLock: &sync.Mutex{},
		logger:   logger,
	}

	model, err := newArm.flxArmModel()
	if err != nil {
		return nil, err
	}
	newArm.model = model

	mp, err := motionplan.NewCBiRRTMotionPlanner(newArm.model, 4, logger)
	if err != nil {
		return nil, err
	}
	newArm.mp = mp

	return newArm, nil
}

func (flx *flxArm) flxArmModel() (referenceframe.Model, error) {
	model := referenceframe.NewSimpleModel()
	basem, err := referenceframe.UnmarshalModelJSON(flxbotBasejson, "")
	if err != nil {
		return nil, err
	}
	eem, err := referenceframe.UnmarshalModelJSON(flxbotEndEffectorjson, "")
	if err != nil {
		return nil, err
	}

	model.OrdTransforms = append(model.OrdTransforms, basem)

	switch flx.mode {
	case flxfoab:
		flxm, err := referenceframe.UnmarshalModelJSON(flxbotFixedjson, "")
		if err != nil {
			return nil, err
		}
		segm, err := referenceframe.UnmarshalModelJSON(flxbotSegmentjson, "")
		if err != nil {
			return nil, err
		}
		model.OrdTransforms = append(model.OrdTransforms, segm, flxm, eem)
		return model, nil
	case flxseg:
		segm, err := referenceframe.UnmarshalModelJSON(flxbotSegmentjson, "")
		if err != nil {
			return nil, err
		}
		for idx := 0; idx < flx.numSeg; idx++ {
			model.OrdTransforms = append(model.OrdTransforms, segm)
		}
		model.OrdTransforms = append(model.OrdTransforms, eem)
		return model, nil
	default:
		return nil, errors.Errorf("flxbot Mode %s, not implemented", flx.mode)
	}
}

func (flx *flxArm) GetEndPosition(ctx context.Context) (*commonpb.Pose, error) {
	joints, err := flx.GetJointPositions(ctx)
	if err != nil {
		return nil, err
	}
	return motionplan.ComputePosition(flx.mp.Frame(), joints)
}

func (flx *flxArm) MoveToJointPositions(ctx context.Context, newPositions *pb.JointPositions) error {
	// Os.exec for move to joint positions
	return nil
}

func (flx *flxArm) GetJointPositions(ctx context.Context) (*pb.JointPositions, error) {
	// Code for os exec get joint positions
	return &pb.JointPositions{}, nil
}

func (flx *flxArm) MoveToPosition(ctx context.Context, pos *commonpb.Pose, worldState *commonpb.WorldState) error {
	_, err := flx.GetJointPositions(ctx)
	if err != nil {
		return err
	}

	return nil
}

func (flx *flxArm) ModelFrame() referenceframe.Model {
	return flx.model
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
