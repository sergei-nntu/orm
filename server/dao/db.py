import os
import sqlite3
from contextlib import closing


class Db:

    def __init__(self, db_path, schema_path):
        self.db_path = db_path
        self.schema_path = schema_path

    def execute(self, query, params=None, is_many=False):
        with closing(sqlite3.connect(self.db_path)) as conn:
            try:
                with conn:
                    with closing(conn.cursor()) as cursor:  # auto-closes
                        if is_many:
                            cursor.executemany(query, params or [])
                        else:
                            cursor.execute(query, params or [])

                        conn.commit()

                        if (cursor.description):
                            response = cursor.fetchall()
                        else:
                            response = cursor.lastrowid

                        return response
            except Exception as e:
                print('DB Exception', e)
                pass

    def initialize_db(self):
        if not os.path.exists(self.db_path):
            os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
            self.create_schema()

    def create_schema(self):
        try:
            print('Database not found. Creating database from', self.schema_path)

            with open(self.schema_path, 'r') as file:
                schema = file.read()

            statements = schema.split(';')
            for statement in statements:
                statement = statement.strip()
                if statement:
                    self.execute(statement)

            print('Database successfully created!')
        except Exception as e:
            print('DB Exception', e)
            pass
