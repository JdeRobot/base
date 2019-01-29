from jderobot_config import config


def dameRobot():
    cfg_file = "Kibotics.yml"
    cfg = config.load(cfg_file)
    bot = cfg.getProperty('Kibotics.Robot')
    bot = bot.lower()

    if bot == "pibot":
        # Import real Pibot wrapper
        from real import PiBot as pireal

        robot = pireal(cfg)

    if bot == "mbot":
        #port = cfg.getProperty('Kibotics.Real.Port')
        #robot = MBot()
        None

    elif bot == "gazebo":
        # Import simulated Pibot wrapper
        from gazebo import PiBot as pisim

        robot = pisim(cfg)

    return robot

