from orm_http_server import *
def program_main(should_terminate_function, set_active_block_id, publish_grip_state):
  while True:
    set_active_block_id('T;l+gU?91.)Grg{%n5.a')
    orm_blockly_set_position(0, 0.2, 0.4, 0, 0, 0)
    if should_terminate_function(): return
    set_active_block_id('8i[P23l6AH8g^TzMOhm?')
    orm_blockly_delay(1000)
    if should_terminate_function(): return
    set_active_block_id('=S[h!3e=r}Mn86n]}|*7')
    state = orm_blockly_set_gripper_state(20)
    publish_grip_state(state)
    if should_terminate_function(): return
    set_active_block_id('hbYV{hVQRg*e0RDi5TbD')
    orm_blockly_set_position(0.1, 0.2, 0.4, 0, 0, 0)
    if should_terminate_function(): return
    set_active_block_id('Oy:?K3/py]gN@GLI6%`_')
    orm_blockly_delay(1000)
    if should_terminate_function(): return
    set_active_block_id('1w*n9%f~;7i-P#sRZ]3,')
    state = orm_blockly_set_gripper_state(10)
    publish_grip_state(state)
    if should_terminate_function(): return
