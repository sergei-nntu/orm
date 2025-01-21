from flask import request
from types import SimpleNamespace

from dao import BlocklyDAO


def update_program():
    try:
        data = request.get_json()
        id = request.args.get('id')

        # TODO: add validation
        program_data = {
            'name': data.get('program_name'),
            'description': data.get('program_description'),
            'structure': data.get('program_structure'),
            'is_running': data.get('is_running'),
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