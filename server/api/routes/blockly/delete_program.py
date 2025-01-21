from flask import request

from dao import BlocklyDAO

# TODO: test delete functionality much deeper
def delete_program():
    try:
        id = request.args.get('id')

        dao = BlocklyDAO()
        dao.delete_program(id)

        # TODO: check on correct removing of the record

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