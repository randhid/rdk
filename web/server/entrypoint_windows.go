//go:build windows

package server

import (
	"go.viam.com/rdk/gostream"
)

func makeStreamConfig() gostream.StreamConfig {
	var streamConfig gostream.StreamConfig
	// TODO(RSDK-1771): support video on windows
	return streamConfig
}
