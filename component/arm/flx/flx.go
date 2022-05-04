package flx

import (
	"context"
	"sync"

	// for embedding model kinematics file.
	_ "embed"

	"github.com/edaniels/golog"
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

// AttrConfig is used for converting attributes
type AttrConfig struct {
	Host string `json:"host"`
	NumFlxSeg string `json:"number_of_flx_segments"`
	Mode string `json:"flxbot_mode"`
}


//go:embed flxbot_base_SVA.json
var flxbotBasejson []byte

//go:embed flxbot_segment_SVA.json
var flxbotSegmentjson []byte

//go:embed flxbot_end_effector_SVA.json
var flxbotEndEffectorjson []byte

//go:embed flxbot_fixed_segment_SVA.json
var flxbotFixedjson []byte

type flxArm struct {
	generic.Unimplemented
	host     string
	model    referenceframe.Model
	mp       motionplan.MotionPlanner
	moveLock *sync.Mutex
	logger   golog.Logger

	frameJSON []byte
}

func init() {
	registry.RegisterComponent(arm.Subtype, modelname, registry.Component{
		Constructor: func(
			ctx context.Context,
			r robot.Robot,
			config config.Component,
			logger golog.Logger) (interface{}, error) {
			return NewFLX(ctx, config, logger)
		},
	})

	config.RegisterComponentAttributeMapConverter(arm.SubtypeName, modelname,
		func(attributes config.AttributeMap) (interface{}, error) {
			var conf AttrConfig
			return config.TransformAttributeMapToStruct(&conf, attributes)
		},
		&AttrConfig{})
}

func NewFLX(ctx context.Context, cfg config.Component, logger golog.Logger) (arm.Arm, error) {
	flxconf := cfg.ConvertedAttributes.(*AttrConfig)


	flxArm := &flxArm{
		Unimplemented: generic.Unimplemented{},
		host:          flxconf.Host,
		model:         nil,
		mp:            nil,
		moveLock:      &sync.Mutex{},
		logger:        logger,
		frameJSON:     []byte{},
	}
	return flxArm, nil
}	

func flxArmModel(flxbotMode string) (referenceframe.Model, error) {
	return referenceframe.UnmarshalModelJSON(flxbotSegmentjson)
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
	return nil
}

func (flx *flxArm) MoveToJointPositions(ctx context.Context, newPositions *pb.JointPositions) error {
	return nil
}

func (flx *flxArm) GetJointPositions(ctx context.Context) (*pb.JointPositions, error) {
	return nil, nil
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
