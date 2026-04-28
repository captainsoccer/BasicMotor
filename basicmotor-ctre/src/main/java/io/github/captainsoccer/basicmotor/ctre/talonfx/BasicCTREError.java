package io.github.captainsoccer.basicmotor.ctre.talonfx;

import com.ctre.phoenix6.StatusCode;
import io.github.captainsoccer.basicmotor.BasicError;

public class BasicCTREError implements BasicError {
    private final StatusCode error;

    public BasicCTREError(StatusCode statusCode){
        error = statusCode;
    }

    @Override
    public boolean isError() {
        return error.isError();
    }

    @Override
    public boolean isOk() {
        return error.isOK();
    }

    @Override
    public int getID() {
        return error.value;
    }

    @Override
    public String getName() {
        return error.getName();
    }
}
