from jderobot_config import config


def dameRobot():
    cfg_file = "JdeRobotKids.yml"
    cfg = config.load(cfg_file)
    bot = cfg.getProperty('JdeRobotKids.Robot')
    bot = bot.lower()

    if bot == "pibot":
        # Import real Pibot wrapper
        from real import PiBot as pireal

        robot = pireal(cfg)

    if bot == "mbot":
        port = cfg.getProperty('JdeRobotKids.Real.Port')
        #robot = MBot()

    elif bot == "gazebo":
        # Import simulated Pibot wrapper
        from gazebo import PiBot as pisim

        robot = pisim(cfg)

    return robot

