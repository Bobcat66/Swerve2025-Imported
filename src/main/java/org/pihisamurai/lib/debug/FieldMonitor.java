package org.pihisamurai.lib.debug;

import java.lang.reflect.Field;

/**
 * A reference to a field within a particular object. This is meant to make it easier to monitor the state of a private field
 * 
 * WARNING: ABSOLUTELY NO GUARANTEES ARE PROVIDED REGARDING MEMORY OR NULL-POINTER SAFETY. USE AT YOUR OWN RISK
 * @param <T> the type of the field being referenced
 */
@SuppressWarnings("unchecked")
public class FieldMonitor<T> {
    private final Field field; //The field being referenced
    private final Object instance; //The object being monitored
    public FieldMonitor(Field field, Object instance){
        field.setAccessible(true);
        this.field = field;
        this.instance = instance;
    }

    /** returns the value of the monitored field */
    public T get() {
        try{
            return (T)field.get(instance);
        } catch (IllegalAccessException e){
            e.printStackTrace();
            return null;
        }
    }

    /** 
     * Sets the value of the monitored field
     * 
     * @return a boolean indicating the success of the operation
     */
    public boolean set(T newVal) {
        try {
            field.set(instance,newVal);
            return true;
        } catch (IllegalAccessException e) {
            e.printStackTrace();
            return false;
        }
    }
}
