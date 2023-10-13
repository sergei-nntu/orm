from orm_http_server import *
def program_main(should_terminate_function, set_active_block_id, publish_grip_state):
  while True:
    set_active_block_id('RFGzLt$^9EfYqP*sXsXO')
    orm_blockly_set_position(0, 0.2, 0.4, 0, 0, 0)
    if should_terminate_function(): return
    set_active_block_id('EBL!-BF*Eqe2#b7D7C16')
    orm_blockly_delay(1000)
    if should_terminate_function(): return
    set_active_block_id('^4m6vrMSVOAj{4hQ{1f-')
    state = orm_blockly_set_gripper_state(90)
    publish_grip_state(state)
    if should_terminate_function(): return
    set_active_block_id('-AGbJ.0)idwf{Z50S/rB')
    orm_blockly_delay(1000)
    if should_terminate_function(): return
    set_active_block_id(']bXV;5YHMYfc+Ky,dMTn')
    state = orm_blockly_set_gripper_state(20)
    publish_grip_state(state)
    if should_terminate_function(): return
    set_active_block_id('(rQYsbZO?11~If}vfT_W')
    orm_blockly_delay(1000)
    if should_terminate_function(): return
    set_active_block_id('NJ|bpi0M8qzPEb7WJ8nE')
    orm_blockly_set_position(0.1, 0.2, 0.4, 0, 0, 0)
    if should_terminate_function(): return
