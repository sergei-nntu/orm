from flask import request

from dao import BlocklyDAO


def start_program():
    data = request.get_json()
    id = data.get('id')

    dao = BlocklyDAO()
    dao.get_program(id)

    # Algorithm
    # 1) Validate the input data
    # 2) Get the program from the data base by id
    # 3) Launch the program in the tread
    return 'Start selected program'