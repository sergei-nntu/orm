from api.utils import JsonResponse
from dao import BlocklyDAO


def get_program(id):
    try:
        dao = BlocklyDAO()
        program_data = dao.get_program(id)

        if not program_data:
             return JsonResponse(
                status="error",
                message=f"No program found with the provided ID"
            ).to_json(400)
        
        response = JsonResponse(status='success', data=program_data, message='Request processed successfully')
        return response.to_json(200)

    except KeyError as e:
            error_message = f"Data access error: {str(e)}"
            response = JsonResponse(status="error", message="Data access error", error=error_message)
            return response.to_json(500)

    except Exception as e:
        error_message = f"An unexpected error occurred: {str(e)}"
        response = JsonResponse(status="error", message="Unexpected error", error=error_message)
        return response.to_json(500)