from flask import request
from types import SimpleNamespace

from dao import BlocklyDAO
from api.utils import JsonResponse


def update_program():
    try:
        data = request.get_json()

        id = request.args.get('id')
        program_name = data.get('program_name')
        program_description = data.get('program_description')
        program_structure = data.get('program_structure')

        if not id:
            return JsonResponse(
                status="error",
                message=f"The program id should be provided as a query parameter - /blockly?id=0"
            ).to_json(400)

        if not program_name or not program_description or not program_structure:
            return JsonResponse(
                status="error",
                message=f"Both 'program_name', 'program_description' and 'program_structure' must be provided in the request"
            ).to_json(400)

        program_data = {
            'name': data.get('program_name'),
            'description': data.get('program_description'),
            'structure': data.get('program_structure'),
        }

        program = SimpleNamespace(**program_data)

        dao = BlocklyDAO()
        dao.update_program(program, id)

        response = JsonResponse(status='success', data={}, message='Request processed successfully')
        return response.to_json(200)

    except KeyError as e:
        error_message = f"Data access error: {str(e)}"
        response = JsonResponse(status="error", message="Data access error", error=error_message)
        return response.to_json(500)

    except Exception as e:
        error_message = f"An unexpected error occurred: {str(e)}"
        response = JsonResponse(status="error", message="Unexpected error", error=error_message)
        return response.to_json(500)