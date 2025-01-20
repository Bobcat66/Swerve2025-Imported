package org.pihisamurai.lib.debug;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.List;

public class ReflectedClosure<T> {
    private final Class<?>[] paramTypes;
    private final Method boundMethod;
    private final Object boundEnv;

    public ReflectedClosure(Object env, Method method, Class<?>... paramTypes){
        this.paramTypes = paramTypes;
        this.boundMethod = method;
        this.boundEnv = env;
    }

    @SuppressWarnings("unchecked")
    public T invoke(Object... params) {

        //Checks parameters for correctness
        if (params.length != paramTypes.length) {
            throw new IllegalArgumentException("Invalid Number of Arguments");
        }
        for (int i = 0 ; i < params.length ; i++) {
            try {
                paramTypes[i].cast(params[i]);
            } catch (ClassCastException e){
                throw e;
            }
        }

        try {
            return (T)boundMethod.invoke(boundEnv,params);
        } catch (InvocationTargetException | IllegalAccessException e) {
            e.printStackTrace();
            return null;
        }
    }
}