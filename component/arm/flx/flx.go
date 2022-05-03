package flx

import (
	"context"
	"sync"

	// for embedding model kinematics file.
	_ "embed"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/component/arm"
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

//go:embed flx_kinematics_SVA.json
var flxmodeljson []byte

// AttrConfig is used for converting attributes
type AttrConfig struct {
	Host string `json:"host"`
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

type flxData struct {
}

type flx struct {
	host     string
	model    referenceframe.Model
	mp       motionplan.MotionPlanner
	moveLock *sync.Mutex
	logger   golog.Logger

	frameJSON []byte
}

func NewFLX(ctx context.Context, cfg config.Component, logger golog.Logger) (arm.Arm, error) {
	flxconf := cfg.ConvertedAttributes.(*AttrConfig)
	flxArm := &flx{
		host: flxconf.Host,
	}
	return flxArm, nil
}

func (flx *flx) ModelFrame() referenceframe.Model {
	return flx.model
}

func (flx *flx) GetEndPosition(ctx context.Context) (*commonpb.Pose, error) {
	return nil, nil
}

func (flx *flx) MoveToPosition(ctx context.Context, pos *commonpb.Pose, worldState *commonpb.WorldState) error {
	return nil
}

func (flx *flx) MoveToJointPositions(ctx context.Context, newPositions *pb.JointPositions) error {
	return nil
}

func (flx *flx) GetJointPositions(ctx context.Context) (*pb.JointPositions, error) {
	return nil, nil
}

func (flx *flx) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return nil, nil
}

func (flx *flx) GoToInputs(ctx context.Context, goal []referenceframe.Input) error {
	return nil
}

func (flx *flx) Close() {}
