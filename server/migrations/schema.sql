DROP TABLE IF EXISTS programs;

CREATE TABLE programs (
  id INTEGER PRIMARY KEY AUTOINCREMENT,

  program_name TEXT UNIQUE NOT NULL,
  program_description TEXT DEFAULT NULL,
  program_structure TEXT NOT NULL,

  is_running INTEGER NOT NULL DEFAULT 0 CHECK (is_running IN (0, 1)),
  
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- TODO: maybe it would be pretty nice to have version like this - version INTEGER DEFAULT 1 -
-- to control program versions (need create a new table with this specific name program_versions)
-- an exmaple of the program_verions talble:
-- CREATE TABLE program_versions (
--     id INTEGER PRIMARY KEY AUTOINCREMENT,
--     program_id INTEGER NOT NULL,
--     version INTEGER NOT NULL,
--     block_structure TEXT NOT NULL,
--     updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
--     FOREIGN KEY (program_id) REFERENCES programs (id)
-- );
--
-- TODO: create trigger to update the 'updated_at' field