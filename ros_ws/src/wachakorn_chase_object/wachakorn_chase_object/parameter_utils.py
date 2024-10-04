from rclpy.node import Parameter, SetParametersResult


def validate_positive_double(parameter: Parameter) -> SetParametersResult:
    if parameter.type_ == Parameter.Type.DOUBLE:
        parameter_value = parameter.get_parameter_value().double_value

        if parameter_value <= 0:
            return SetParametersResult(
                successful=False,
                reason=f"Expected '{parameter.name}' value to be positive, but got {parameter_value}",
            )

        return SetParametersResult(successful=True)
    else:
        return SetParametersResult(
            successful=False,
            reason=f"Expected ' {parameter.name}' to be Double, got {parameter.type_}",
        )


def validate_positive_int(parameter: Parameter) -> SetParametersResult:
    if parameter.type_ == Parameter.Type.INTEGER:
        parameter_value = parameter.get_parameter_value().integer_value

        if parameter_value <= 0:
            return SetParametersResult(
                successful=False,
                reason=f"Expected '{parameter.name}' value to be positive, but got {parameter_value}",
            )

        return SetParametersResult(successful=True)
    else:
        return SetParametersResult(
            successful=False,
            reason=f"Expected ' {parameter.name}' to be Integer, got {parameter.type_}",
        )
