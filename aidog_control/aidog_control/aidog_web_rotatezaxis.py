#!/usr/bin/env python3


from ros2web_interfaces.srv import HTTP
from ros2web_interfaces.msg import ContentType, BodyPart
from aidog_control.aidog_rotatezaxis import RotateZAxis
import json
import numpy as  np
import urllib.parse
   
class RotateZAxisWebService(RotateZAxis):
    def __init__(self, name, absolute=False):
        super().__init__(name, absolute=absolute)
        self.rad2deg = 180.0 / np.pi
        self.deg2rad = np.pi / 180.0
    
    def make_response(self, jsonresponse, httpresponse):
        if jsonresponse['success'] is False:
            httpresponse.status = 400
        else:
            httpresponse.status = 200
        httpresponse.content_type = ContentType.APPLICATION_JSON
        if 'last_angle' in jsonresponse:
            jsonresponse['last_angle'] *= self.rad2deg
        httpresponse.text = json.dumps(jsonresponse)
        return httpresponse
    
    def process_post_request(self, request: HTTP.Request, response: HTTP.Response):
        if request.content_type == ContentType.APPLICATION_JSON:
            try:
                body = json.loads(request.text)
                turn_angle = body.get('turn_angle', 30) * self.deg2rad
                angular_velocity = body.get('angular_velocity', 1) * self.deg2rad
                if self.absolute:
                    # Absolute rotation - ignore start_angle and end_angle
                    start_angle = 0.0
                    end_angle = 0.0
                else:
                    # Relative rotation - use start_angle and end_angle
                    start_angle = body.get('current_angle', 0.0) * self.deg2rad
                    end_angle = body.get('end_angle', 2.0) * self.deg2rad
            except json.JSONDecodeError as e:
                return self.make_response({'success': False, 'message': str(e)}, response)
        elif request.content_type == ContentType.APPLICATION_X_WWW_FORM_URLENCODED:
            # Handle multipart form data
            angular_velocity = 0.0
            turn_angle = 0.0
            start_angle = 0.0
            end_angle = 0.0

            if request.multipart is None or len(request.multipart) == 0:
                return self.make_response({'success': False, 'message': 'No body parts found'}, response)
            body_parts: list[BodyPart] = request.multipart
            for part in body_parts:
                if part.name == 'turn_angle':
                    turn_angle = float(bytearray(list(part.data))) * self.deg2rad
                elif part.name == 'angular_velocity':
                    angular_velocity = float(bytearray(list(part.data))) * self.deg2rad
                elif part.name == 'current_angle':
                    start_angle = float(bytearray(list(part.data))) * self.deg2rad
                elif part.name == 'end_angle':
                    end_angle = float(bytearray(list(part.data))) * self.deg2rad
        else:
            return self.make_response({'success': False, 'message': 'Invalid content type'}, response)
        result =  self.handle_rotatezaxis(turn_angle, angular_velocity, start_angle, end_angle)
        return self.make_response(result, response)
          
    def process_get_request(self, request: HTTP.Request, response: HTTP.Response):
        self.get_logger().info('/get: received request')
        query = dict(urllib.parse.parse_qsl(request.query))
        turn_angle = float(query.get('turn_angle', 30))
        angular_velocity = float(query.get('angular_velocity', 5))
        self.get_logger().info('/get: turn_angle {:.2f} angular_velocity'.format(turn_angle, angular_velocity))
        turn_angle *= self.deg2rad
        angular_velocity *= self.deg2rad
        if self.absolute:
            # Absolute rotation - ignore start_angle and end_angle
            start_angle = 0.0
            end_angle = 0.0
        else:
            # Relative rotation - use start_angle and end_angle
            start_angle = float(query.get('current_angle', 0.0)) * self.deg2rad
            end_angle = float(query.get('end_angle', 180.0)) * self.deg2rad
        result = self.handle_rotatezaxis(turn_angle, angular_velocity, start_angle, end_angle)
        return self.make_response(result, response)
