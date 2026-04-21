from fastapi import Request

from src.app.robot_controller import RobotController


def get_controller(request: Request) -> RobotController:
    controller: RobotController = request.app.state.controller
    return controller
