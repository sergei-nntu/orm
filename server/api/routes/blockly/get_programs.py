from flask import jsonify

from dao import BlocklyDAO
from api.utils import JsonResponse


def get_programs():
    try:
        dao = BlocklyDAO()
        programs_data = dao.get_programs()

        response = JsonResponse(
            status="success",
            data=programs_data,
            message="Request processed successfully",
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
