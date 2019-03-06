// adds Custom Gauss blocks + Python generators

var Blockly = require('node-blockly/python');

/*
 *  Blocks definition
 */

//↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 运动部分 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

Blockly.Blocks['gauss_move_joints'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Move Joints");
    this.appendDummyInput()
      .appendField("joint1")
      .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.0001), "JOINTS_1")
      .appendField("joint2")
      .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.0001), "JOINTS_2")
      .appendField("joint3")
      .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.0001), "JOINTS_3")
      .appendField("joint4")
      .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.0001), "JOINTS_4")
      .appendField("joint5")
      .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.0001), "JOINTS_5")
      .appendField("joint6")
      .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.0001), "JOINTS_6");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#009FAB");
    this.setTooltip("Give all 6 joints to move the robot");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_move_pose'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Move Pose");
    this.appendDummyInput()
      .appendField("x")
      .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.0001), "POSE_X")
      .appendField("y")
      .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.0001), "POSE_Y")
      .appendField("z")
      .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.0001), "POSE_Z")
      .appendField("roll")
      .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.0001), "POSE_ROLL")
      .appendField("pitch")
      .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.0001), "POSE_PITCH")
      .appendField("yaw")
      .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.0001), "POSE_YAW");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#009FAB");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_shift_pose'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Shift");
    this.appendDummyInput()
      .appendField(new Blockly.FieldDropdown([["pos. x", "0"], ["pos. y", "1"], ["pos. z", "2"], ["rot. x", "3"], ["rot. y", "4"], ["rot. z", "5"]]), "SHIFT_POSE_AXIS")
      .appendField("by")
      .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.0001), "SHIFT_POSE_VALUE");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#009FAB");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_set_arm_max_speed'] = {
  init: function () {
    this.appendValueInput("SET_ARM_MAX_SPEED")
      .setCheck("Number")
      .appendField("Set Arm max. speed to");
    this.appendDummyInput()
      .appendField("%");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#009FAB");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

//↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 基础部分 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

Blockly.Blocks['gauss_calibrate_auto'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Calibrate motors (auto)");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#080D5E");
    this.setTooltip("Will auto calibrate motors. If already calibrated, will do nothing.");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_calibrate_manual'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Calibrate motors (manual)");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#080D5E");
    this.setTooltip("Will manually calibrate motors (robot needs to be in home position). If already calibrated, will do nothing.");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_activate_learning_mode'] = {
  init: function () {
    this.appendDummyInput()
      .appendField(new Blockly.FieldDropdown([["Activate", "1"], ["Deactivate", "0"]]), "LEARNING_MODE_VALUE")
      .appendField("learning mode");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#080D5E");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_sleep'] = {
  init: function () {
    this.appendValueInput("SLEEP_TIME")
      .setCheck("Number")
      .appendField("Wait for ");
    this.appendDummyInput()
      .appendField("seconds");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#080D5E");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_comment'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Comment :")
      .appendField(new Blockly.FieldTextInput(""), "COMMENT_TEXT");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#080D5E");
    this.setTooltip("This block will not be executed.");
    this.setHelpUrl("");
  }
};

//↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 工具部分 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

Blockly.Blocks['gauss_open_gripper'] = {
  init: function () {
    this.appendValueInput("OPEN_GRIPPER_ID")
      .setCheck("gauss_tool_select")
      .appendField("Open Gripper");
    this.appendDummyInput()
      .appendField("at speed")
      .appendField(new Blockly.FieldDropdown([["1/5", "100"], ["2/5", "250"], ["3/5", "500"], ["4/5", "750"], ["5/5", "1000"]]), "OPEN_SPEED");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#005359");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_close_gripper'] = {
  init: function () {
    this.appendValueInput("CLOSE_GRIPPER_ID")
      .setCheck("gauss_tool_select")
      .appendField("Close Gripper");
    this.appendDummyInput()
      .appendField("at speed")
      .appendField(new Blockly.FieldDropdown([["1/5", "100"], ["2/5", "250"], ["3/5", "500"], ["4/5", "750"], ["5/5", "1000"]]), "CLOSE_SPEED");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#005359");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_change_tool'] = {
  init: function () {
    this.appendValueInput("NEW_TOOL_ID")
      .setCheck("gauss_tool_select")
      .appendField("Change tool to");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#005359");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_tool_select'] = {
  init: function () {
    this.appendDummyInput()
      .appendField(new Blockly.FieldDropdown([["gripper 1", "TOOL_GRIPPER_1_ID"], ["electromagnet 1", "TOOL_ELECTROMAGNET_1_ID"], ["vacuum pump 1", "TOOL_VACUUM_PUMP_1_ID"], ["laser 1", "TOOL_LASER_1_ID"], ["dc motor 1", "TOOL_DC_MOTOR_1_ID"]]), "TOOL_SELECT");
    this.setOutput(true, "gauss_tool_select");
    this.setColour("#005359");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_detach_tool'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Detach current tool");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#005359");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_pull_air_vacuum_pump'] = {
  init: function () {
    this.appendValueInput("PULL_AIR_VACUUM_PUMP_ID")
      .setCheck("gauss_tool_select")
      .appendField("Pull air with Vacuum Pump");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#005359");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_push_air_vacuum_pump'] = {
  init: function () {
    this.appendValueInput("PUSH_AIR_VACUUM_PUMP_ID")
      .setCheck("gauss_tool_select")
      .appendField("Push air with Vacuum Pump");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#005359");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_setup_electromagnet'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Setup Electromagnet")
      .appendField(new Blockly.FieldDropdown([["electromagnet 1", "TOOL_ELECTROMAGNET_1_ID"]]), "SETUP_ELECTROMAGNET_ID")
      .appendField(new Blockly.FieldDropdown([["2C", "GPIO_2C"], ["2D", "GPIO_2D"]]), "SETUP_ELECTROMAGNET_PIN");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#005359");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_setup_laser'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Setup Laser")
      .appendField(new Blockly.FieldDropdown([["laser 1", "TOOL_LASER_1_ID"]]), "SETUP_LASER_ID")
      .appendField(new Blockly.FieldDropdown([["1A", "GPIO_1A"], ["1B", "GPIO_1B"]]), "SETUP_LASER_PIN");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#005359");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_setup_dc_motor'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Setup DC Motor")
      .appendField(new Blockly.FieldDropdown([["dc motor 1", "TOOL_DC_MOTOR_1_ID"]]), "SETUP_DC_MOTOR_ID")
      .appendField(new Blockly.FieldDropdown([["1C", "GPIO_1C"]]), "SETUP_DC_MOTOR_PIN");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#005359");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_activate_electromagnet'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Active Electromagnet")
      .appendField(new Blockly.FieldDropdown([["electromagnet 1", "TOOL_ELECTROMAGNET_1_ID"]]), "ACTIVE_ELECTROMAGNET_ID")
      .appendField(new Blockly.FieldDropdown([["2C", "GPIO_2C"], ["2D", "GPIO_2D"]]), "ACTIVE_ELECTROMAGNET_PIN");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#005359");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_deactivate_electromagnet'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Deactive Electromagnet")
      .appendField(new Blockly.FieldDropdown([["electromagnet 1", "TOOL_ELECTROMAGNET_1_ID"]]), "DEACTIVE_ELECTROMAGNET_ID")
      .appendField(new Blockly.FieldDropdown([["2C", "GPIO_2C"], ["2D", "GPIO_2D"]]), "DEACTIVE_ELECTROMAGNET_PIN");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#005359");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_activate_laser'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Active Laser")
      .appendField(new Blockly.FieldDropdown([["laser 1", "TOOL_LASER_1_ID"]]), "ACTIVATE_LASER_ID")
      .appendField(new Blockly.FieldDropdown([["1A", "GPIO_1A"], ["1B", "GPIO_1B"]]), "ACTIVATE_LASER_PIN");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#005359");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_deactivate_laser'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("dective Laser")
      .appendField(new Blockly.FieldDropdown([["laser 1", "TOOL_LASER_1_ID"]]),"DEACTIVATE_LASER_ID")
      .appendField(new Blockly.FieldDropdown([["1A", "GPIO_1A"], ["1B", "GPIO_1B"]]), "DEACTIVATE_LASER_PIN");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#005359");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_activate_dc_motor'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Active DC Motor")
      .appendField(new Blockly.FieldDropdown([["dc motor 1", "TOOL_DC_MOTOR_1_ID"]]), "ACTIVATE_DC_MOTOR_ID")
      .appendField(new Blockly.FieldDropdown([["1C", "GPIO_1C"]]), "ACTIVATE_DC_MOTOR_PIN");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#005359");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_deactivate_dc_motor'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Deactive DC Motor")
      .appendField(new Blockly.FieldDropdown([["dc motor 1", "TOOL_DC_MOTOR_1_ID"]]), "DEACTIVATE_DC_MOTOR_ID")
      .appendField(new Blockly.FieldDropdown([["1C", "GPIO_1C"]]), "DEACTIVATE_DC_MOTOR_PIN");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#005359");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

//↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ I/O 部分 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

Blockly.Blocks['gauss_gpio_select'] = {
  init: function () {
    this.appendDummyInput()
      .appendField(new Blockly.FieldDropdown([["1A", "GPIO_1A"], ["1B", "GPIO_1B"], ["1C", "GPIO_1C"], ["2A", "GPIO_2A"], ["2B", "GPIO_2B"], ["2C", "GPIO_2C"], ["2D", "GPIO_2D"]]), "GPIO_SELECT");
    this.setOutput(true, "gauss_gpio_select");
    this.setColour("#008891");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_set_pin_mode'] = {
  init: function () {
    this.appendValueInput("SET_PIN_MODE_PIN")
      .setCheck("gauss_gpio_select")
      .setAlign(Blockly.ALIGN_RIGHT)
      .appendField("Set Pin");
    this.appendDummyInput()
      .appendField("to mode")
      .appendField(new Blockly.FieldDropdown([["INPUT", "PIN_MODE_INPUT"], ["OUTPUT", "PIN_MODE_OUTPUT"]]), "PIN_MODE_SELECT");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#008891");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_digital_write'] = {
  init: function () {
    this.appendValueInput("DIGITAL_WRITE_PIN")
      .setCheck("gauss_gpio_select")
      .appendField("Set Pin");
    this.appendDummyInput()
      .appendField("to state")
      .appendField(new Blockly.FieldDropdown([["HIGH", "PIN_HIGH"], ["LOW", "PIN_LOW"]]), "PIN_WRITE_SELECT");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour("#008891");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_digital_read'] = {
  init: function () {
    this.appendValueInput("DIGITAL_READ_PIN")
      .setCheck("gauss_gpio_select")
      .appendField("Get Pin");
    this.appendDummyInput()
      .appendField("state");
    this.setInputsInline(true);
    this.setOutput(true, "gauss_gpio_state");
    this.setColour("#008891");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['gauss_gpio_state'] = {
  init: function () {
    this.appendDummyInput()
      .appendField("state")
      .appendField(new Blockly.FieldDropdown([["HIGH", "PIN_HIGH"], ["LOW", "PIN_LOW"]]), "GPIO_STATE_SELECT");
    this.setOutput(true, "gauss_gpio_state");
    this.setColour("#008891");
    this.setTooltip("");
    this.setHelpUrl("");
  }
};

/*
 * Generators
 */

Blockly.Python['gauss_move_joints'] = function (block) {
  var number_joints_1 = block.getFieldValue('JOINTS_1');
  var number_joints_2 = block.getFieldValue('JOINTS_2');
  var number_joints_3 = block.getFieldValue('JOINTS_3');
  var number_joints_4 = block.getFieldValue('JOINTS_4');
  var number_joints_5 = block.getFieldValue('JOINTS_5');
  var number_joints_6 = block.getFieldValue('JOINTS_6');

  var code = 'n.move_joints([' + number_joints_1 + ', ' + number_joints_2 + ', '
    + number_joints_3 + ', ' + number_joints_4 + ', ' + number_joints_5 + ', ' + number_joints_6 + '])\n'
  return code;
};

Blockly.Python['gauss_move_pose'] = function (block) {
  var number_pose_x = block.getFieldValue('POSE_X');
  var number_pose_y = block.getFieldValue('POSE_Y');
  var number_pose_z = block.getFieldValue('POSE_Z');
  var number_pose_roll = block.getFieldValue('POSE_ROLL');
  var number_pose_pitch = block.getFieldValue('POSE_PITCH');
  var number_pose_yaw = block.getFieldValue('POSE_YAW');

  var code = 'n.move_pose(' + number_pose_x + ', ' + number_pose_y + ', ' +
    number_pose_z + ', ' + number_pose_roll + ', ' + number_pose_pitch +
    ', ' + number_pose_yaw + ")\n";
  return code;
};

Blockly.Python['gauss_shift_pose'] = function (block) {
  var dropdown_shift_pose_axis = block.getFieldValue('SHIFT_POSE_AXIS');
  var number_shift_pose_value = block.getFieldValue('SHIFT_POSE_VALUE');

  var code = 'n.shift_pose(' + dropdown_shift_pose_axis + ', ' +
    number_shift_pose_value + ')\n';
  return code;
};

Blockly.Python['gauss_set_arm_max_speed'] = function(block) {
  var value_set_arm_max_speed = Blockly.Python.valueToCode(block, 'SET_ARM_MAX_SPEED', Blockly.Python.ORDER_ATOMIC) || '0';
  var code = 'n.set_arm_max_velocity(' + value_set_arm_max_speed + ')\n';
  return code;
};

Blockly.Python['gauss_calibrate_auto'] = function (block) {
  var code = 'n.calibrate_auto()\n';
  return code;
};

Blockly.Python['gauss_calibrate_manual'] = function (block) {
  var code = 'n.calibrate_manual()\n';
  return code;
};

Blockly.Python['gauss_activate_learning_mode'] = function (block) {
  var dropdown_learning_mode_value = block.getFieldValue('LEARNING_MODE_VALUE');
  var code = 'n.activate_learning_mode(' + dropdown_learning_mode_value + ')\n';
  return code;
};

Blockly.Python['gauss_sleep'] = function (block) {
  var value_sleep_time = Blockly.Python.valueToCode(block, 'SLEEP_TIME', Blockly.Python.ORDER_ATOMIC) || '0';
  var code = 'n.wait(' + value_sleep_time + ')\n';
  return code;
};

Blockly.Python['gauss_open_gripper'] = function (block) {
  var value_gripper_id = Blockly.Python.valueToCode(block, 'OPEN_GRIPPER_ID', Blockly.Python.ORDER_ATOMIC) || '(TOOL_NONE)';
  value_gripper_id = value_gripper_id.replace('(', '').replace(')', '');
  var number_open_speed = block.getFieldValue('OPEN_SPEED');
  var code = 'n.open_gripper(' + value_gripper_id + ', ' + number_open_speed + ')\n';
  return code;
};

Blockly.Python['gauss_close_gripper'] = function (block) {
  var value_gripper_id = Blockly.Python.valueToCode(block, 'CLOSE_GRIPPER_ID', Blockly.Python.ORDER_ATOMIC) || '(TOOL_NONE)';
  value_gripper_id = value_gripper_id.replace('(', '').replace(')', '');
  var number_close_speed = block.getFieldValue('CLOSE_SPEED');
  var code = 'n.close_gripper(' + value_gripper_id + ', ' + number_close_speed + ')\n';
  return code;
};

Blockly.Python['gauss_change_tool'] = function (block) {
  var value_tool_name = Blockly.Python.valueToCode(block, 'NEW_TOOL_ID', Blockly.Python.ORDER_ATOMIC) || '(TOOL_NONE)';
  var code = 'n.change_tool' + value_tool_name + '\n';
  return code;
};

Blockly.Python['gauss_tool_select'] = function (block) {
  var dropdown_tool_select = block.getFieldValue('TOOL_SELECT');
  var code = dropdown_tool_select;
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['gauss_detach_tool'] = function (block) {
  var code = 'n.change_tool(0)\n';
  return code;
};

Blockly.Python['gauss_pull_air_vacuum_pump'] = function (block) {
  var value_vacuum_pump_id = Blockly.Python.valueToCode(block, 'PULL_AIR_VACUUM_PUMP_ID', Blockly.Python.ORDER_ATOMIC) || '(TOOL_NONE)';
  var code = 'n.pull_air_vacuum_pump' + value_vacuum_pump_id + '\n';
  return code;
}

Blockly.Python['gauss_push_air_vacuum_pump'] = function (block) {
  var value_vacuum_pump_id = Blockly.Python.valueToCode(block, 'PUSH_AIR_VACUUM_PUMP_ID', Blockly.Python.ORDER_ATOMIC) || '(TOOL_NONE)';
  var code = 'n.push_air_vacuum_pump' + value_vacuum_pump_id + '\n';
  return code;
};

Blockly.Python['gauss_setup_electromagnet'] = function (block) {
  var value_electromagnet_id = block.getFieldValue('SETUP_ELECTROMAGNET_ID');
  var value_electromagnet_pin = block.getFieldValue('SETUP_ELECTROMAGNET_PIN');
  var code = 'n.setup_electromagnet(' + value_electromagnet_id + ', ' + value_electromagnet_pin + ')\n';
  return code;
};

Blockly.Python['gauss_setup_laser'] = function (block) {
  var value_laser_id = block.getFieldValue('SETUP_LASER_ID');
  var value_laser_pin = block.getFieldValue('SETUP_LASER_PIN');
  var code = 'n.setup_laser(' + value_laser_id + ', ' + value_laser_pin + ')\n';
  return code;
};

Blockly.Python['gauss_setup_dc_motor'] = function (block) {
  var value_dc_motor_id = block.getFieldValue('SETUP_DC_MOTOR_ID');
  var value_dc_motor_pin = block.getFieldValue('SETUP_DC_MOTOR_PIN');  
  var code = 'n.setup_dc_motor(' + value_dc_motor_id + ', ' + value_dc_motor_pin + ')\n';
  return code;
};

Blockly.Python['gauss_activate_electromagnet'] = function (block) {
  var value_electromagnet_id = block.getFieldValue('ACTIVE_ELECTROMAGNET_ID');
  var value_electromagnet_pin = block.getFieldValue('ACTIVE_ELECTROMAGNET_PIN');
  var code = 'n.activate_electromagnet(' + value_electromagnet_id + ', ' + value_electromagnet_pin + ')\n';
  return code;
};

Blockly.Python['gauss_deactivate_electromagnet'] = function (block) {
  var value_electromagnet_id = block.getFieldValue('DEACTIVE_ELECTROMAGNET_ID');
  var value_electromagnet_pin = block.getFieldValue('DEACTIVE_ELECTROMAGNET_PIN');
  var code = 'n.deactivate_electromagnet(' + value_electromagnet_id + ', ' + value_electromagnet_pin + ')\n';
  return code;
};

Blockly.Python['gauss_activate_laser'] = function (block) {
  var value_laser_id = block.getFieldValue('ACTIVATE_LASER_ID');
  var value_laser_pin = block.getFieldValue('ACTIVATE_LASER_PIN');
  var code = 'n.activate_laser(' + value_laser_id + ', ' + value_laser_pin + ')\n';
  return code;
};

Blockly.Python['gauss_deactivate_laser'] = function (block) {
  var value_laser_id = block.getFieldValue('DEACTIVATE_LASER_ID');
  var value_laser_pin = block.getFieldValue('DEACTIVATE_LASER_PIN');
  var code = 'n.deactivate_laser(' + value_laser_id + ', ' + value_laser_pin + ')\n';
  return code;
};

Blockly.Python['gauss_activate_dc_motor'] = function (block) {
  var value_dc_motor_id = block.getFieldValue('ACTIVATE_DC_MOTOR_ID');
  var value_dc_motor_pin = block.getFieldValue('ACTIVATE_DC_MOTOR_PIN');
  var code = 'n.activate_dc_motor(' + value_dc_motor_id + ', ' + value_dc_motor_pin + ')\n';
  return code;
};

Blockly.Python['gauss_deactivate_dc_motor'] = function (block) {
  var value_dc_motor_id = block.getFieldValue('DEACTIVATE_DC_MOTOR_ID');
  var value_dc_motor_pin = block.getFieldValue('DEACTIVATE_DC_MOTOR_PIN');
  var code = 'n.deactivate_dc_motor(' + value_dc_motor_id + ', ' + value_dc_motor_pin + ')\n';
  return code;
};

Blockly.Python['gauss_gpio_select'] = function (block) {
  var dropdown_gpio_select = block.getFieldValue('GPIO_SELECT');
  var code = dropdown_gpio_select;
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['gauss_set_pin_mode'] = function (block) {
  var value_pin = Blockly.Python.valueToCode(block, 'SET_PIN_MODE_PIN', Blockly.Python.ORDER_ATOMIC) || '(0)';
  var dropdown_pin_mode_select = block.getFieldValue('PIN_MODE_SELECT');
  value_pin = value_pin.replace('(', '').replace(')', '');
  var code = 'n.pin_mode(' + value_pin + ', ' + dropdown_pin_mode_select + ')\n';
  return code;
};

Blockly.Python['gauss_digital_write'] = function (block) {
  var value_pin = Blockly.Python.valueToCode(block, 'DIGITAL_WRITE_PIN', Blockly.Python.ORDER_ATOMIC) || '(0)';
  var dropdown_pin_write_select = block.getFieldValue('PIN_WRITE_SELECT');
  value_pin = value_pin.replace('(', '').replace(')', '');
  var code = 'n.digital_write(' + value_pin + ', ' + dropdown_pin_write_select + ')\n';
  return code;
};

Blockly.Python['gauss_digital_read'] = function (block) {
  var value_pin = Blockly.Python.valueToCode(block, 'DIGITAL_READ_PIN', Blockly.Python.ORDER_ATOMIC) || '(0)';
  var code = 'n.digital_read' + value_pin;
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['gauss_gpio_state'] = function (block) {
  var dropdown_gpio_state_select = block.getFieldValue('GPIO_STATE_SELECT');
  var code = dropdown_gpio_state_select;
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['gauss_comment'] = function(block) {
  var text_comment_text = block.getFieldValue('COMMENT_TEXT');
  var code = '# ' + text_comment_text + '\n';
  return code;
};

/*
 * Export module
 */

module.exports =  {
  Blockly: Blockly,
};

