package debug;

import java.lang.ClassLoader;
import java.security.ProtectionDomain;

/** For exposing protected methods to the reflection API */
public class DebugClassLoader extends ClassLoader {
    public final static String name = "Debug ClassLoader";
    public Class<?> toClass(String name,byte[] bytecode, int off, int len, ProtectionDomain domain){
        return super.defineClass(name, bytecode, off, len, domain);
    }
}
