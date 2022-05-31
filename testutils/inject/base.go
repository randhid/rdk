package inject

import (
	"context"

	"go.viam.com/utils"

	"go.viam.com/rdk/component/base"
)

// Base is an injected base.
type Base struct {
	base.LocalBase
	DoFunc           func(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error)
	MoveStraightFunc func(ctx context.Context, distanceMm int, mmPerSec float64) error
	SpinFunc         func(ctx context.Context, angleDeg float64, degsPerSec float64) error
	GetWidthFunc     func(ctx context.Context) (int, error)
	StopFunc         func(ctx context.Context) error
	CloseFunc        func(ctx context.Context) error
}

// MoveStraight calls the injected MoveStraight or the real version.
func (b *Base) MoveStraight(ctx context.Context, distanceMm int, mmPerSec float64) error {
	if b.MoveStraightFunc == nil {
		return b.LocalBase.MoveStraight(ctx, distanceMm, mmPerSec)
	}
	return b.MoveStraightFunc(ctx, distanceMm, mmPerSec)
}

// Spin calls the injected Spin or the real version.
func (b *Base) Spin(ctx context.Context, angleDeg float64, degsPerSec float64) error {
	if b.SpinFunc == nil {
		return b.LocalBase.Spin(ctx, angleDeg, degsPerSec)
	}
	return b.SpinFunc(ctx, angleDeg, degsPerSec)
}

// GetWidth calls the injected GetWidth or the real version.
func (b *Base) GetWidth(ctx context.Context) (int, error) {
	if b.GetWidthFunc == nil {
		return b.LocalBase.GetWidth(ctx)
	}
	return b.GetWidthFunc(ctx)
}

// Stop calls the injected Stop or the real version.
func (b *Base) Stop(ctx context.Context) error {
	if b.StopFunc == nil {
		return b.LocalBase.Stop(ctx)
	}
	return b.StopFunc(ctx)
}

// Close calls the injected Close or the real version.
func (b *Base) Close(ctx context.Context) error {
	if b.CloseFunc == nil {
		return utils.TryClose(ctx, b.LocalBase)
	}
	return b.CloseFunc(ctx)
}

// Do calls the injected Do or the real version.
func (b *Base) Do(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if b.DoFunc == nil {
		return b.LocalBase.Do(ctx, cmd)
	}
	return b.DoFunc(ctx, cmd)
}
