from flask import request

from dao import BlocklyDAO
from api.utils import JsonResponse


def start_program():
    try:
        data = request.get_json()
        id = data.get('id')

        # 1) Validate the input data
        if not id:
            return JsonResponse(
                status="error",
                message=f"The program ID was not provided in the request"
            ).to_json(400)

        # 2) Get the program from the data base by id
        dao = BlocklyDAO()
        program_data = dao.get_program(id)

        # 3) Validate the program data from dao
        if not program_data:
             return JsonResponse(
                status="error",
                message=f"No program found with the provided ID"
            ).to_json(400)

        print(data)
        # Algorithm
        # 3) Launch the program in the tread

        response = JsonResponse(status='success', data=data, message='Request processed successfully')
        return response.to_json(200)

    except KeyError as e:
            error_message = f"Data access error: {str(e)}"
            response = JsonResponse(status="error", message="Data access error", error=error_message)
            return response.to_json(500)

    except Exception as e:
        error_message = f"An unexpected error occurred: {str(e)}"
        response = JsonResponse(status="error", message="Unexpected error", error=error_message)
        return response.to_json(500)