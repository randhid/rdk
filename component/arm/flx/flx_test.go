package flx

import (
	"testing"

	"go.viam.com/test"

	"go.viam.com/rdk/referenceframe"
)

func TestNewArm(t *testing.T) {
	testmod := referenceframe.NewSimpleModel()

	f1, err := referenceframe.UnmarshalModelJSON(flxbotSegmentjson, "")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, f1, test.ShouldHaveSameTypeAs, testmod)

	f2, err := referenceframe.UnmarshalModelJSON(flxbotFixedjson, "")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, f2, test.ShouldHaveSameTypeAs, testmod)

	f3, err := referenceframe.UnmarshalModelJSON(flxbotBasejson, "")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, f3, test.ShouldHaveSameTypeAs, testmod)

	f4, err := referenceframe.UnmarshalModelJSON(flxbotEndEffectorjson, "")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, f4, test.ShouldHaveSameTypeAs, testmod)
}
