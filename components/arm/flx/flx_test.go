package flx

import (
	"context"
	"testing"

	"github.com/edaniels/golog"
	"go.viam.com/test"

	"go.viam.com/rdk/config"
	commonpb "go.viam.com/rdk/proto/api/common/v1"
	componentpb "go.viam.com/rdk/proto/api/component/arm/v1"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/utils"
)

func TestFlxBotSVAFiles(t *testing.T) {
	testmod := referenceframe.NewSimpleModel()

	f1, err := referenceframe.UnmarshalModelJSON(flxbotSegmentjson, "")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, f1, test.ShouldHaveSameTypeAs, testmod)

	f2, err := referenceframe.UnmarshalModelJSON(flxbotFixedjson, "")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, f2, test.ShouldHaveSameTypeAs, testmod)

	// f3, err := referenceframe.UnmarshalModelJSON(flxbotBasejson, "")
	// test.That(t, err, test.ShouldBeNil)
	// test.That(t, f3, test.ShouldHaveSameTypeAs, testmod)

	// f4, err := referenceframe.UnmarshalModelJSON(flxbotEndEffectorjson, "")
	// test.That(t, err, test.ShouldBeNil)
	// test.That(t, f4, test.ShouldHaveSameTypeAs, testmod)
}

func TestKinematics(t *testing.T) {
	ctx := context.Background()
	logger := golog.NewTestLogger(t)
	cfg, err := config.Read(ctx, utils.ResolveFile("samples/flx/flx.json"), logger)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg, test.ShouldNotBeNil)

	arm, err := newFLX(ctx, cfg.Components[0], logger)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, arm, test.ShouldNotBeNil)

	model := arm.ModelFrame()
	test.That(t, model, test.ShouldHaveSameTypeAs, referenceframe.NewSimpleModel())
	dof := len(model.DoF())
	t.Log("model", model.DoF())

	flx := arm.(*flxArm)
	goal := commonpb.Pose{X: 5, Y: 5}
	t.Log("motionplan", flx.mp)

	pose, err := flx.mp.Plan(ctx, &goal, referenceframe.JointPosToInputs(&componentpb.JointPositions{Degrees: make([]float64, dof)}), nil)
	t.Log(pose)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pose, test.ShouldNotBeNil)
}
