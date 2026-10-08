// Custom implementation incorporating pair key-value mapping of 6328 Mechanical
// Advantage for on-the-fly updating auton selection code.
package frc.robot.util.auto

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StringPublisher
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

class AutonSelector<V>(
    private val key: String,
    defaultName: String,
    defaultQuestions: List<AutoQuestion<V>>,
    defaultCommands: () -> Command
) : SubsystemBase() {
    private val routineChooser: LoggedDashboardChooser<AutoRoutine<V>> = LoggedDashboardChooser("$key/Routine")
    private val questionPublishers: MutableList<StringPublisher>
    private val questionChoosers: MutableList<LoggedDashboardChooser<String>>

    private var lastRoutine: AutoRoutine<V>
    private var lastResponses: List<V>

    init {
        val defaultRoutine = AutoRoutine(defaultName, defaultQuestions, defaultCommands)
        routineChooser.addDefaultOption(defaultRoutine.name, defaultRoutine)
        lastRoutine = defaultRoutine
        lastResponses = defaultQuestions.map { it.responses.values.first() }

        // Publish questions and choosers
        questionPublishers = mutableListOf()
        questionChoosers = mutableListOf()
        for (i in 0..<MAX_QUESTIONS) {
            val publisher =
                NetworkTableInstance.getDefault().getStringTopic("/SmartDashboard/$key/Question #${i + 1}").publish()
            publisher.set("N/A")
            questionPublishers.add(publisher)
            questionChoosers.add(LoggedDashboardChooser("$key/Question #${i + 1} Chooser"))
            questionChoosers[i].addDefaultOption("N/A", "N/A")
        }
        updateQuestions(defaultQuestions)
    }

    /** Registers a new auto routine that can be selected.  */
    fun addRoutine(name: String, questions: List<AutoQuestion<V>>, command: () -> Command) {
        if (questions.size > MAX_QUESTIONS) {
            throw RuntimeException(
                "Auto routine contained more than $MAX_QUESTIONS questions: $name"
            )
        }
        routineChooser.addOption(name, AutoRoutine(name, questions, command))
    }

    val command: Command
        /** Returns the selected auto command.  */
        get() = lastRoutine.command()

    val responses: List<V>
        /** Returns the selected question responses.  */
        get() = lastResponses.toList()

    override fun periodic() {
        // Skip updates when actively running in auto
        if (DriverStation.isAutonomousEnabled()) {
            return
        }

        // Update the list of questions
        val selectedRoutine = routineChooser.get() ?: return
        if (selectedRoutine != lastRoutine) {
            updateQuestions(selectedRoutine.questions)
        }

        // Update the routine and responses
        lastRoutine = selectedRoutine
        lastResponses = lastRoutine.questions.mapIndexed { i, question ->
            val responseString = questionChoosers[i].get()
            question.responses[responseString] ?: question.responses.values.first()
        }
    }

    private fun updateQuestions(questions: List<AutoQuestion<V>>) {
        for (i in 0..<MAX_QUESTIONS) {
            questionChoosers[i] = LoggedDashboardChooser("$key/Question #${i + 1} Chooser")
            val chooser = questionChoosers[i]
            if (i < questions.size) {
                val question = questions[i]
                questionPublishers[i].set(question.question)
                question.responses.keys.forEachIndexed { optionIndex, option ->
                    if (optionIndex == 0) chooser.addDefaultOption(option, option)
                    else chooser.addOption(option, option)
                }
            } else {
                questionPublishers[i].set("N/A")
                chooser.addDefaultOption("N/A", "N/A")
            }
        }
    }

    /** A customizable auto routine associated with a single command.  */
    private data class AutoRoutine<V>(
        val name: String, val questions: List<AutoQuestion<V>>, val command: () -> Command
    )

    /** A question to ask for customizing an auto routine.  */
    @JvmRecord
    data class AutoQuestion<V>(val question: String, val responses: Map<String, V>) {
        companion object {
            fun makeQuestion(question: String, responses: List<String>): AutoQuestion<Any> = AutoQuestion(
                question, mutableMapOf(*responses.map { it to it }.toTypedArray())
            )
        }
    }

    companion object {
        private const val MAX_QUESTIONS = 4
    }
}
