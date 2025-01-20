package org.pihisamurai.lib.debug;

import java.util.function.Function;

/** 
 * Monitors a reference to an object using the reflection API, for debugging.
 * NOTE: A monad is a monoid in the category of endofunctors
 * @param T the type of the object being monitored
 * @author Jesse Kane
 */
public class ObjectMonitor<T> {
    private final T object;
    private final Class<?> clazz;
    private final ClassMonitor clazzMonitor;
    private ObjectMonitor(T object) {
        this.object = object;
        this.clazz = object.getClass();
        this.clazzMonitor = ReflectionDebugger.getInstance().getClassMonitor(this.clazz);
    }
    /** Wraps an object inside an objectMonitor */
    public static <U> ObjectMonitor<U> of(U object){
        return new ObjectMonitor<>(object);
    }

    /**returns the object*/
    public T get() {
        return object;
    }
    
    /** Flat Map */
    public <R> ObjectMonitor<R> fmap(Function<T,ObjectMonitor<R>> mapper) {
        return mapper.apply(object);
    }

    /** Regular Map */
    public <R> ObjectMonitor<R> map(Function<T,R> mapper) {
        return ObjectMonitor.of(mapper.apply(object));
    }

    /** Gets field from the monitored object */
    public <U> ObjectMonitor<U> getField(String fieldName){
        return clazzMonitor.getObjectMonitorFromField(fieldName, object);
    }

    /** Gets closure from a method of the monitored object*/
    public <U> ReflectedClosure<U> getClosure(String methodName){
        return clazzMonitor.getClosure(methodName, object);
    }

}
