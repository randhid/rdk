package main

import (
	"context"

	"github.com/edaniels/golog"
	"go.viam.com/utils"

	"go.viam.com/rdk/config"
	robotimpl "go.viam.com/rdk/robot/impl"
	"go.viam.com/rdk/robot/web"
)

var logger = golog.NewDevelopmentLogger("flxbot")

func main() {
	utils.ContextualMain(mainWithArgs, logger)
}

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) error {
	cfg, err := config.Read(ctx, "samples/flx/flx.json", logger)
	if err != nil {
		return err
	}

	myRobot, err := robotimpl.RobotFromConfigPath(ctx, "samples/flx/flx.json", logger)
	if err != nil {
		return err
	}
	defer myRobot.Close(ctx)

	return web.RunWebWithConfig(ctx, myRobot, cfg, logger)
}
