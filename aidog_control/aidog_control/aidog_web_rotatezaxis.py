#!/usr/bin/env python3


from ros2web_interfaces.srv import HTTP
from ros2web_interfaces.msg import ContentType, BodyPart
from aidog_control.aidog_rotatezaxis import RotateZAxis
import json
import urllib.parse
   
class RotateZAxisWebService(RotateZAxis):
    def __init__(self, name, absolute=False):
        super().__init__(name, absolute=absolute)
    
    def make_response(self, jsonresponse, httpresponse):
        if jsonresponse['success'] is False:
            httpresponse.status = 400
        else:
            httpresponse.status = 200
        httpresponse.content_type = ContentType.APPLICATION_JSON
        httpresponse.text = json.dumps(jsonresponse)
        return httpresponse
    
    def process_get_request(self, request: HTTP.Request, response: HTTP.Response):
        if request.content_type == ContentType.APPLICATION_JSON or request.content_type == ContentType.APPLICATION_JSON_UTF8:
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
        else:
            # Handle multipart form data
            angular_velocity = 0.0
            turn_angle = 0.0
            start_angle = 0.0
            end_angle = 0.0
            if request.multipart is None or len(request.multipart) == 0:
                return self.make_response({'success': False, 'message': 'No multipart data found'}, response)
            if request.multipart[0].content_type != ContentType.MULTIPART_FORM_DATA:
                return self.make_response({'success': False, 'message': 'Invalid content type'}, response)
        body_parts: list[BodyPart] = request.multipart
        if len(body_parts) == 0:
            return self.make_response({'success': False, 'message': 'No body parts found'}, response)
        for part in body_parts:
            if part.name == 'turn_angle':
                turn_angle = float(bytearray(list(part.data))) * self.deg2rad
            elif part.name == 'angular_velocity':
                angular_velocity = float(bytearray(list(part.data))) * self.deg2rad
            elif part.name == 'current_angle':
                start_angle = float(bytearray(list(part.data))) * self.deg2rad
            elif part.name == 'end_angle':
                end_angle = float(bytearray(list(part.data))) * self.deg2rad
        result =  self.handle_rotatezaxis(turn_angle, angular_velocity, start_angle, end_angle)
        return self.make_response(result, response)
          
    def process_post_request(self, request: HTTP.Request, response: HTTP.Response):
        query = dict(urllib.parse.parse_qsl(request.query))
        turn_angle = float(query.get('turn_angle', 30)) * self.deg2rad
        angular_velocity = float(query.get('angular_velocity', 1)) * self.deg2rad
        if self.absolute:
            # Absolute rotation - ignore start_angle and end_angle
            start_angle = 0.0
            end_angle = 0.0
        else:
            # Relative rotation - use start_angle and end_angle
            start_angle = float(query.get('current_angle', 0.0)) * self.deg2rad
            end_angle = float(query.get('end_angle', 2.0)) * self.deg2rad
        result = self.handle_rotatezaxis(turn_angle, angular_velocity, start_angle, end_angle)
        return self.make_response(result, response)
