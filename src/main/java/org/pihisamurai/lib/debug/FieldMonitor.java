package org.pihisamurai.lib.debug;

import java.lang.reflect.Field;

/**
 * A reference to a field within an object. This is meant to make it easier to monitor the state of a private field.
 * 
 * The term "value" refers to the object contained within the monitored field
 * 
 * WARNING: ABSOLUTELY NO GUARANTEES ARE PROVIDED REGARDING MEMORY OR NULL-POINTER SAFETY. USE AT YOUR OWN RISK
 * @param <T> the type of the field being referenced
 * @deprecated replaced by {@link org.pihisamurai.lib.debug.ObjectMonitor}
 */
@SuppressWarnings("unchecked")
public class FieldMonitor<T> {
    private final Field field; //The field being monitored
    private final Object instance; //The object that owns the monitored field
    private final Class<?> clazz; //The class of the value
    private final ClassMonitor clazzMonitor; //A classMonitor of the value's class
    public FieldMonitor(Field field, Object instance){
        field.setAccessible(true);
        this.field = field;
        this.instance = instance;
        this.clazz = get().getClass();
        this.clazzMonitor = ReflectionDebugger.getInstance().getClassMonitor(clazz);
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

    /** returns a monitor of one of the value's fields*/
    public <U> FieldMonitor<U> getSubMonitor(String subfieldName) {
        return this.clazzMonitor.<U>getFieldMonitor(subfieldName, get());
    }

    /** returns a closure of one of the value's methods */
    public <U> ReflectedClosure<U> getClosure(String methodName) {
        return this.clazzMonitor.<U>getClosure(methodName, get());
    }
    /** returns a closure of one of the monitored object's static methods */
    public <U> ReflectedClosure<U> getStaticClosure(String methodName) {
        return this.clazzMonitor.<U>getClosure(methodName,null);
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
