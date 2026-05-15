import com.squareup.kotlinpoet.*
import java.lang.reflect.Modifier
import java.net.URLClassLoader
import org.gradle.api.DefaultTask
import org.gradle.api.file.ConfigurableFileCollection
import org.gradle.api.file.DirectoryProperty
import org.gradle.api.provider.Property
import org.gradle.api.tasks.*

abstract class GenerateUnitTask extends DefaultTask {
    @Input abstract Property<String> getSourceClassName()
    @Input abstract Property<String> getOutputNamespace()
    
    // We use the project's compile classpath to find the external JAR
    @InputFiles 
    @PathSensitive(PathSensitivity.RELATIVE)
    abstract ConfigurableFileCollection getClasspath()
    
    @OutputDirectory abstract DirectoryProperty getOutputDir()

    @TaskAction
    void generate() {
        String className = sourceClassName.get()
        String outPkg = outputNamespace.get()
        
        // 1. Create a ClassLoader from the provided classpath (JARs)
        def urls = classpath.files.collect { it.toURI().toURL() } as java.net.URL[]
        def classLoader = new URLClassLoader(urls, getClass().classLoader)
        
        Class<?> registryClass = classLoader.loadClass(className)

        FileSpec.Builder fileBuilder = FileSpec.builder(outPkg, "${registryClass.simpleName}")
                .addFileComment("Generated from external class: $className")

        registryClass.getDeclaredFields().each { field ->
            if (Modifier.isStatic(field.modifiers) && Modifier.isPublic(field.modifiers)) {
                String extensionName = formatExtensionName(field.name)
                Class<?> returnClass = field.type.getMethod("of", double.class).getReturnType()
                if (returnClass.simpleName != "Measure") {
                    ClassName ktReturnType = new ClassName(
                        returnClass.getPackage().getName(), 
                        returnClass.getSimpleName()
                    )

                    fileBuilder.addImport(registryClass, field.name);

                    [TypeNames.DOUBLE, TypeNames.INT].each { receiver ->
                        PropertySpec prop = PropertySpec.builder(extensionName, ktReturnType)
                                .receiver(receiver)
                                .getter(FunSpec.getterBuilder()
                                        .addStatement("return %N.of(this.toDouble())", field.name)
                                        .addModifiers(KModifier.INLINE)
                                        .build())
                                .build()
                        fileBuilder.addProperty(prop)
                    }
                }
            }
        }
        fileBuilder.build().writeTo(outputDir.get().asFile)
    }

    String formatExtensionName(String name) {
        if (name == name.toUpperCase()) return name.toLowerCase()
        return name.substring(0, 1).toLowerCase() + name.substring(1)
    }
}