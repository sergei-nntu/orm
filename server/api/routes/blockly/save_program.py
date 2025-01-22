from flask import request
from types import SimpleNamespace

from dao import BlocklyDAO
from api.utils import JsonResponse


def save_program():
    try:
        data = request.get_json()

        program_name = data.get('program_name')
        program_description = data.get('program_description')
        program_structure = data.get('program_structure')

        if not program_name or not program_description or not program_structure:
            return JsonResponse(
                status="error",
                message=f"Both 'program_name', 'program_description' and 'program_structure' must be provided in the request"
            ).to_json(400)

        program_data = {
            'name': program_name,
            'description': program_description,
            'structure': program_structure,
        }

        program = SimpleNamespace(**program_data)

        dao = BlocklyDAO()
        dao.add_program(program)

        response = JsonResponse(status='success', data={}, message='Request processed successfully')
        return response.to_json(201)

    except KeyError as e:
        error_message = f"Data access error: {str(e)}"
        response = JsonResponse(status="error", message="Data access error", error=error_message)
        return response.to_json(500)

    except Exception as e:
        error_message = f"An unexpected error occurred: {str(e)}"
        response = JsonResponse(status="error", message="Unexpected error", error=error_message)
        return response.to_json(500)
