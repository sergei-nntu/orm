from flask import g

class BlocklyDAO:
    def __init__(self):
        self.db = g.db

    def add_program(self, data):
        query = 'INSERT INTO programs (program_name, program_description, program_structure) VALUES (?, ?, ?)'
        return self.db.execute(query, (data.name, data.description, data.structure))

    def get_program(self, id):
        query = 'SELECT id, program_name, program_description, program_structure FROM programs WHERE id = ?'
        return self.db.execute(query, (id,))

    def get_programs(self):
        query = 'SELECT id, program_name, program_description, program_structure FROM programs'
        return self.db.execute(query)

    def update_program(self, data, id):
        updates, params = [], []

        if data.name is not None:
            updates.append("program_name = ?")
            params.append(data.name)
        if data.description is not None:
            updates.append("program_description = ?")
            params.append(data.description)
        if data.structure is not None:
            updates.append("program_structure = ?")
            params.append(data.structure)

        params.append(id)

        query = f"UPDATE programs SET {', '.join(updates)} WHERE id = ?"
        return self.db.execute(query, params)

    def delete_program(self, id):
        query = 'DELETE FROM programs WHERE id = ?'
        return self.db.execute(query, (id,))
