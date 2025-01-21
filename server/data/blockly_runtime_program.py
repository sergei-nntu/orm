def program(should_terminate_function, set_active_block_id, publish_grip_state, orm_blockly_delay, orm_blockly_set_position, orm_blockly_set_gripper_state):
  while True:
    set_active_block_id('cRuxvDvvbkVA#4mUL}tZ')
    state = orm_blockly_set_gripper_state(20)
    publish_grip_state(state)
    if should_terminate_function(): return
    set_active_block_id('dns+Gk%R`NYP/et6SNip')
    orm_blockly_delay(1000)
    if should_terminate_function(): return
