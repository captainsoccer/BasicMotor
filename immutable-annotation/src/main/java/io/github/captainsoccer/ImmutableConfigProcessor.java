package io.github.captainsoccer;

import com.google.auto.service.AutoService;
import com.palantir.javapoet.*;

import javax.annotation.processing.*;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.*;
import javax.tools.Diagnostic;
import java.io.IOException;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.Optional;
import java.util.Set;

/**
 * Immutable config processor creates a copy of the annotated class and transforms all the fields to final.
 * This is used in creating the immutable config files needed in the basic motor
 */
@AutoService(Processor.class)
public class ImmutableConfigProcessor extends AbstractProcessor {

    /**
     * marks a class as needing of an immutable version
     */
    @Retention(RetentionPolicy.CLASS)
    @Target(ElementType.TYPE)
    public @interface Immutable{}

    @Override
    public SourceVersion getSupportedSourceVersion(){
        return SourceVersion.latestSupported();
    }

    @Override
    public Set<String> getSupportedAnnotationTypes() {
        return Set.of("io.github.captainsoccer.ImmutableConfigProcessor.Immutable");
    }

    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        Optional<? extends TypeElement> annotationOptional = annotations.stream()
                .filter((te) -> te.getSimpleName().toString().equals("Immutable")).findFirst();

        if(annotationOptional.isEmpty()) return false;

        var annotation = annotationOptional.get();

        roundEnv.getElementsAnnotatedWith(annotation).forEach(this::createImmutableClass);

        return true;
    }

    /**
     * Creates the actual immutable class
     * @param classElement the class to take the fields and name from
     */
    private void createImmutableClass(Element classElement){
        String className = "Immutable" + classElement.getSimpleName();
        String classPackage = getPackageName(classElement);

        var typeBuilder = TypeSpec.classBuilder(className).addModifiers(Modifier.PUBLIC);

        var methodBuilder = MethodSpec.constructorBuilder();

        String paramName = unCapitalize(classElement.getSimpleName().toString());

        methodBuilder.addParameter(TypeName.get(classElement.asType()), paramName);

        classElement.getEnclosedElements().stream()
                .filter((element) -> element.getKind().isField()).forEach(
                        (field) -> {
                            typeBuilder.addField(
                                    TypeName.get(field.asType()), field.getSimpleName().toString(), Modifier.PUBLIC, Modifier.FINAL
                            );

                            methodBuilder.addCode(
                                    "this." + field.getSimpleName() + " = " + paramName + "." + field.getSimpleName() + ";\n"
                            );
                        }
                );

        typeBuilder.addMethod(methodBuilder.build());

        JavaFile file = JavaFile.builder(classPackage, typeBuilder.build()).build();

        try {
            file.writeTo(processingEnv.getFiler());
        } catch (IOException e) {
            processingEnv
                    .getMessager()
                    .printMessage(Diagnostic.Kind.ERROR, "Failed to write class", classElement);
            e.  printStackTrace();
        }
    }

    /**
     * makes the first letter of the string lowerCase.
     * makes the class name the name for the parameter of the class.
     * @param input the name of the class
     * @return the name of the parameter of the class
     */
    private static String unCapitalize(String input) {
        if (input == null || input.isEmpty()) {
            return input;
        }
        return input.substring(0, 1).toLowerCase() + input.substring(1);
    }

    /**
     * gets the package of the given class.
     * @param e the class
     * @return the package of the class
     */
    private static String getPackageName(Element e) {
        while (e != null) {
            if (e.getKind().equals(ElementKind.PACKAGE)) {
                return ((PackageElement) e).getQualifiedName().toString();
            }
            e = e.getEnclosingElement();
        }

        return null;
    }
}
