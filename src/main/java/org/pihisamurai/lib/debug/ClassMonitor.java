package org.pihisamurai.lib.debug;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;

public class ClassMonitor {
    public Map<String,Field> fields = new HashMap<>();
    public Map<String,Method> methods = new HashMap<>();
    public Class<?> clazz;
    public ClassMonitor(Class<?> clazz) {
        this.clazz = clazz;
    }
    public <U> FieldMonitor<U> getFieldMonitor(String fieldName,Object instance) {
        return new FieldMonitor<U>(fields.get(fieldName),instance);
    }

    @SuppressWarnings("unchecked")
    public <U> ObjectMonitor<U> getObjectMonitorFromField(String fieldName,Object instance) {
        try {
            return ObjectMonitor.of((U)fields.get(fieldName).get(instance));
        } catch (IllegalAccessException e) {
            e.printStackTrace();
            return null;
        }
    }

    public <U> ReflectedClosure<U> getClosure(String methodName,Object instance) {
        Method method = methods.get(methodName);
        return new ReflectedClosure<U>(instance, method, method.getParameterTypes());
    }

    public <U> ReflectedClosure<U> getStaticClosure(String methodName) {
        Method method = methods.get(methodName);
        return new ReflectedClosure<U>(null,method, method.getParameterTypes());
    }
}
