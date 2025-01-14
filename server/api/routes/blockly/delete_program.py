from flask import request

from dao import BlocklyDAO


def delete_program():
    id = request.args.get('id')

    dao = BlocklyDAO()
    dao.delete_program(id)

    return f'Program with id = {id} was deleted\n'