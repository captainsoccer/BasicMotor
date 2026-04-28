package io.github.captainsoccer.basicmotor;

public interface BasicError{
    boolean isError();
    boolean isOk();
    int getID();
    String getName();
}
