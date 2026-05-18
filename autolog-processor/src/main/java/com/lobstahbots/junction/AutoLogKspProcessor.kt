package com.lobstahbots.junction

import com.google.devtools.ksp.processing.Dependencies
import com.google.devtools.ksp.processing.Resolver
import com.google.devtools.ksp.processing.SymbolProcessor
import com.google.devtools.ksp.processing.SymbolProcessorEnvironment
import com.google.devtools.ksp.processing.SymbolProcessorProvider
import com.google.devtools.ksp.symbol.ClassKind
import com.google.devtools.ksp.symbol.KSAnnotated
import com.google.devtools.ksp.symbol.KSClassDeclaration
import com.google.devtools.ksp.symbol.KSPropertyDeclaration
import com.google.devtools.ksp.symbol.KSType
import com.google.devtools.ksp.symbol.Origin
import com.google.devtools.ksp.symbol.Modifier as KModifier
import com.squareup.javapoet.ClassName
import com.squareup.javapoet.JavaFile
import com.squareup.javapoet.MethodSpec
import com.squareup.javapoet.TypeName
import com.squareup.javapoet.TypeSpec
import java.io.OutputStreamWriter
import javax.lang.model.element.Modifier

class AutoLogKspProcessorProvider : SymbolProcessorProvider {
    override fun create(environment: SymbolProcessorEnvironment): SymbolProcessor {
        return AutoLogKspProcessor(environment)
    }
}

class AutoLogKspProcessor(
    private val environment: SymbolProcessorEnvironment
) : SymbolProcessor {
    private val logger = environment.logger
    private val codeGenerator = environment.codeGenerator
    private val generated = mutableSetOf<String>()

    override fun process(resolver: Resolver): List<KSAnnotated> {
        val symbols = AUTOLOG_ANNOTATIONS.asSequence()
            .flatMap { resolver.getSymbolsWithAnnotation(it).asSequence() }
            .toList()
        symbols.filterIsInstance<KSClassDeclaration>()
            .forEach { classDecl ->
                processClass(classDecl)
            }

        return emptyList()
    }

    private fun processClass(classDecl: KSClassDeclaration) {
        if (classDecl.classKind != ClassKind.CLASS) {
            logger.error("@AutoLog can only be applied to classes", classDecl)
            return
        }

        val autologgedClassName = classDecl.simpleName.asString() + "AutoLogged"
        val autologgedPackage = classDecl.packageName.asString()
        val qualifiedName = classDecl.qualifiedName?.asString()
        if (autologgedPackage.isBlank() || qualifiedName == null) {
            logger.error("Failed to determine class name", classDecl)
            return
        }

        val generatedName = "$autologgedPackage.$autologgedClassName"
        if (!generated.add(generatedName)) {
            return
        }

        val toLogBuilder = MethodSpec.methodBuilder("toLog")
            .addAnnotation(Override::class.java)
            .addModifiers(Modifier.PUBLIC)
            .addParameter(LOG_TABLE_TYPE, "table")

        val fromLogBuilder = MethodSpec.methodBuilder("fromLog")
            .addAnnotation(Override::class.java)
            .addModifiers(Modifier.PUBLIC)
            .addParameter(LOG_TABLE_TYPE, "table")

        val cloneBuilder = MethodSpec.methodBuilder("clone")
            .addModifiers(Modifier.PUBLIC)
            .addCode("\$L copy = new \$L();\n", autologgedClassName, autologgedClassName)
            .returns(ClassName.get(autologgedPackage, autologgedClassName))

        var current: KSClassDeclaration? = classDecl
        var isSuperclass = false

        while (current != null) {
            val finalTypeName = current.simpleName.asString()
            val finalIsSuperclass = isSuperclass

            current.declarations.filterIsInstance<KSPropertyDeclaration>().forEach { property ->
                if (finalIsSuperclass && property.modifiers.contains(KModifier.PRIVATE)) {
                    return@forEach
                }

                val simpleName = property.simpleName.asString()
                val logName = simpleName.substring(0, 1).uppercase() + simpleName.substring(1)
                val fieldType = property.type.resolve().toString()
                val typeSuggestion = UNLOGGABLE_TYPES_SUGGESTIONS[fieldType]
                val isKotlinProperty = property.origin == Origin.KOTLIN
                val getterExpr = if (isKotlinProperty) {
                    "${getterName(property)}()"
                } else {
                    simpleName
                }
                val cloneSourceBase = if (isKotlinProperty) {
                    "${getterName(property)}()"
                } else {
                    "this.$simpleName"
                }

                if (typeSuggestion != null ||
                    (fieldType.startsWith("java") && !fieldType.startsWith("java.lang.String"))
                ) {
                    val extraText = if (typeSuggestion != null) {
                        "Did you mean to use \"$typeSuggestion\" instead?"
                    } else {
                        "\"$fieldType\" is not supported"
                    }
                    throw RuntimeException(
                        "[AutoLog] Unkonwn type for \"$simpleName\" from \"$finalTypeName\" ($extraText)"
                    )
                }

                toLogBuilder.addCode("table.put(\$S, \$L);\n", logName, getterExpr)
                if (isKotlinProperty) {
                    if (!property.isMutable) {
                        throw RuntimeException(
                            "[AutoLog] Field \"$simpleName\" from \"$finalTypeName\" must be mutable (var)"
                        )
                    }
                    fromLogBuilder.addCode(
                        "\$L(table.get(\$S, \$L));\n",
                        setterName(property),
                        logName,
                        getterExpr
                    )
                } else {
                    fromLogBuilder.addCode(
                        "\$L = table.get(\$S, \$L);\n",
                        simpleName,
                        logName,
                        simpleName
                    )
                }

                val resolvedType = property.type.resolve()
                val cloneSourceExpr = when {
                    isArrayType(resolvedType) -> "$cloneSourceBase.clone()"
                    resolvedType.declaration.qualifiedName
                        ?.asString()
                        ?.startsWith("edu.wpi.first.units.MutableMeasure") == true ->
                        "$cloneSourceBase.mutableCopy()"
                    else -> cloneSourceBase
                }
                if (isKotlinProperty) {
                    cloneBuilder.addCode("copy.\$L(\$L);\n", setterName(property), cloneSourceExpr)
                } else {
                    cloneBuilder.addCode("copy.\$L = \$L;\n", simpleName, cloneSourceExpr)
                }
            }

            current = resolveSuperclass(current)
            isSuperclass = current != null
        }

        cloneBuilder.addCode("return copy;\n")

        val type = TypeSpec.classBuilder(autologgedClassName)
            .addModifiers(Modifier.PUBLIC)
            .addSuperinterface(LOGGABLE_INPUTS_TYPE)
            .addSuperinterface(ClassName.get("java.lang", "Cloneable"))
            .superclass(ClassName.bestGuess(qualifiedName))
            .addMethod(toLogBuilder.build())
            .addMethod(fromLogBuilder.build())
            .addMethod(cloneBuilder.build())
            .build()

        val file = JavaFile.builder(autologgedPackage, type).build()
        val containingFile = classDecl.containingFile
        val dependencies = if (containingFile != null) {
            Dependencies(false, containingFile)
        } else {
            Dependencies(false)
        }

        try {
            codeGenerator.createNewFile(
                dependencies,
                autologgedPackage,
                autologgedClassName,
                "java"
            ).use { output ->
                OutputStreamWriter(output, Charsets.UTF_8).use { writer ->
                    file.writeTo(writer)
                }
            }
        } catch (e: Exception) {
            logger.error("Failed to write class", classDecl)
            throw e
        }
    }

    private fun resolveSuperclass(classDecl: KSClassDeclaration): KSClassDeclaration? {
        return classDecl.superTypes
            .mapNotNull { it.resolve().declaration as? KSClassDeclaration }
            .firstOrNull { decl ->
                decl.classKind == ClassKind.CLASS &&
                    decl.qualifiedName?.asString() != "kotlin.Any" &&
                    decl.qualifiedName?.asString() != "java.lang.Object"
            }
    }

    private fun isArrayType(type: KSType): Boolean {
        val qualifiedName = type.declaration.qualifiedName?.asString() ?: return false
        return qualifiedName == "kotlin.Array" || qualifiedName.endsWith("Array")
    }

    private fun getterName(property: KSPropertyDeclaration): String {
        val name = property.simpleName.asString()
        return if (isBooleanType(property.type.resolve()) && isIsPrefixedBoolean(name)) {
            name
        } else {
            "get" + name.replaceFirstChar { it.uppercaseChar() }
        }
    }

    private fun setterName(property: KSPropertyDeclaration): String {
        val name = property.simpleName.asString()
        return if (isBooleanType(property.type.resolve()) && isIsPrefixedBoolean(name)) {
            "set" + name.substring(2)
        } else {
            "set" + name.replaceFirstChar { it.uppercaseChar() }
        }
    }

    private fun isIsPrefixedBoolean(name: String): Boolean {
        return name.startsWith("is") && name.length > 2 && name[2].isUpperCase()
    }

    private fun isBooleanType(type: KSType): Boolean {
        val qualifiedName = type.declaration.qualifiedName?.asString() ?: return false
        return qualifiedName == "kotlin.Boolean" ||
            qualifiedName == "java.lang.Boolean" ||
            qualifiedName == "boolean"
    }

    private companion object {
        val AUTOLOG_ANNOTATIONS: Set<String> = setOf(
            "com.lobstahbots.junction.AutoLogKt"
        )

        val LOG_TABLE_TYPE: TypeName =
            ClassName.get("org.littletonrobotics.junction", "LogTable")
        val LOGGABLE_INPUTS_TYPE: TypeName =
            ClassName.get("org.littletonrobotics.junction.inputs", "LoggableInputs")

        val UNLOGGABLE_TYPES_SUGGESTIONS: Map<String, String> = mapOf(
            "java.lang.Byte[]" to "byte[]",
            "java.lang.Boolean" to "boolean",
            "java.lang.Integer" to "int",
            "java.lang.Long" to "long",
            "java.lang.Float" to "float",
            "java.lang.Double" to "double",
            "java.lang.Boolean[]" to "boolean[]",
            "java.lang.Integer[]" to "int[]",
            "java.lang.Long[]" to "long[]",
            "java.lang.Float[]" to "float[]",
            "java.lang.Double[]" to "double[]"
        )
    }
}
