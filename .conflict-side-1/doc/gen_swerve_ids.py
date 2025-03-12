side_converter = {
    0: "Front_Left",
    5: "Front_Right",
    10: "Back_Left",
    15: "Back_Right"
}

type_converter = {
    0: "Drive",
    1: "Controller",
    2: "Steer"
}

def encode_motor_id(side, motor_type):
    side_converter_flipped = {v: k for k, v in side_converter.items()}
    side_num = side_converter_flipped.get(side, "Invalid")

    type_converter_flipped = {v: k for k, v in type_converter.items()}
    type_num = type_converter_flipped.get(motor_type, "Invalid")

    return side_num + type_num

def decode_motor_id(id):
    side_num, type_num = divmod(id)

    return (
        side_converter[side_num],
        type_converter[type_num]
    )
