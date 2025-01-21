from api.utils import JsonResponse
from services import BlocklyService


def get_current_program():
    try:

        blockly = BlocklyService()
        program_structure = blockly.get_program_structure()

        response = JsonResponse(status='success', data=program_structure, message='Request processed successfully')
        return response.to_json(200)

    except KeyError as e:
                error_message = f"Data access error: {str(e)}"
                response = JsonResponse(status="error", message="Data access error", error=error_message)
                return response.to_json(500)

    except Exception as e:
        error_message = f"An unexpected error occurred: {str(e)}"
        response = JsonResponse(status="error", message="Unexpected error", error=error_message)
        return response.to_json(500)