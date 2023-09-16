

def sortMotorReturn2JsonIndex(self, toSort):

    sorted2JsonIndexPositions = [0]*len(self.robotModel)
    for motor_id, motor_position in enumerate(toSort):
        if motor_id in self.motorId2JsonIndex.keys():
            jsonIndex = self.motorId2JsonIndex[motor_id]
            sorted2JsonIndexPositions[jsonIndex] = motor_position
    
    return sorted2JsonIndexPositions

def sortJsonIndex2MotorInput(self, toSort):
    
    sorted2MotorsId = self.motorsCurrentPosition

    for json_id, position in enumerate(toSort):
        if json_id in self.motorId2JsonIndex.values():
            motor_id = self.keyFromValue(self.motorId2JsonIndex, json_id)
            sorted2MotorsId[motor_id] = position
    
    return sorted2MotorsId

def keyFromValue(self, dict, value):
    for key, v in dict.items():
        if v == value:
            return key
    return None

