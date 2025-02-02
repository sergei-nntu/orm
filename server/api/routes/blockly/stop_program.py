from api.utils import JsonResponse
from services import BlocklyService


def stop_program():
    try:
        blockly = BlocklyService()
        blockly.stop_program()

        response = JsonResponse(
            status="success", data={}, message="Request processed successfully"
        )
        return response.to_json(200)

    except KeyError as e:
        error_message = f"Data access error: {str(e)}"
        response = JsonResponse(
            status="error", message="Data access error", error=error_message
        )
        return response.to_json(500)

    except Exception as e:
        error_message = f"An unexpected error occurred: {str(e)}"
        response = JsonResponse(
            status="error", message="Unexpected error", error=error_message
        )
        return response.to_json(500)
