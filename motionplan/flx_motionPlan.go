package motionplan

import (
	"context"
	"math"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	commonpb "go.viam.com/rdk/proto/api/common/v1"
	"go.viam.com/rdk/referenceframe"
	frame "go.viam.com/rdk/referenceframe"
)

const (
	velocity = 0.1
	timeStep = 0.1
	duration = 1
)

type flxMotionPlanner struct {
	vel      float64
	acc      float64
	frame    referenceframe.Frame
	dt       float64
	duration float64
	dof      int
	numMoves int
}

func NewFLXMotionPlanner(frame referenceframe.Frame, nCPU int, logger golog.Logger) (MotionPlanner, error) {

	numMoves := duration / timeStep

	mp := &flxMotionPlanner{
		vel:      velocity,
		dt:       timeStep,
		dof:      len(frame.DoF()),
		duration: duration,
		numMoves: int(numMoves),
	}

	return mp, nil
}

func componentsInPlane(angle float64, plane_normal r3.Vector, pose *commonpb.Pose) error {
	return nil
}

func (flxmp *flxMotionPlanner) Plan(ctx context.Context,
	goal *commonpb.Pose,
	start []frame.Input,
	opts *PlannerOptions,
) ([][]frame.Input, error) {
	startAng := start[0].Value
	endAng := -start[0].Value

	angles := GetAngles(startAng, endAng, flxmp.vel, flxmp.acc, flxmp.dt)

	return [][]frame.Input{frame.FloatsToInputs(angles)}, nil

}

func (flxmp *flxMotionPlanner) Resolution() float64 {
	return flxmp.dt
}

func (flxmp *flxMotionPlanner) Frame() frame.Frame {
	return flxmp.frame
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

func (flxmp *flxMotionPlanner) moveInPlane(startAng, endAng float64) [][]referenceframe.Input {

	angles := GetAngles(startAng, endAng, flxmp.vel, flxmp.acc, flxmp.dt)

	return [][]referenceframe.Input{referenceframe.FloatsToInputs(angles)}

}

func (flxmp *flxMotionPlanner) getPlaneComponents([]float64) []float64 {
	return []float64{}

}
