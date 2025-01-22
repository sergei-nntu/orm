from flask import request

from api.utils import JsonResponse
from services import BlocklyService


def start_program():
    try:
        data = request.get_json()

        program = data.get('program')
        structure = data.get('structure')

        if not program or not structure:
            return JsonResponse(
                status="error",
                message=f"Both 'program' and 'structure' must be provided in the request"
            ).to_json(400)

        blockly = BlocklyService()
        # FIXME: those reload the server 
        blockly.write_program_to_file(program)
        blockly.save_program_structure(structure)

        # TODO: check -> is program started or not
        blockly.start_program()

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
    
    