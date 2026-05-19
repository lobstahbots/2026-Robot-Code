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
    private var lastResponses: MutableMap<String, V>

    init {
        val defaultRoutine = AutoRoutine(defaultName, defaultQuestions, defaultCommands)
        routineChooser.addDefaultOption(defaultRoutine.name, defaultRoutine)
        lastRoutine = defaultRoutine
        lastResponses = mutableMapOf()

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
        get() = lastResponses.values.toList()

    override fun periodic() {
        // Skip updates when actively running in auto
        if (DriverStation.isAutonomousEnabled() && lastRoutine != null && lastResponses == null) {
            return
        }

        // Update the list of questions
        val selectedRoutine = routineChooser.get() ?: return
        if (selectedRoutine != lastRoutine) {
            val questions = selectedRoutine.questions
            for (i in 0..<MAX_QUESTIONS) {
                if (i < questions.size) {
                    questionPublishers[i].set(questions[i].question)
                    // you can't change the options so we just replace the one in the list with a new one and let the old one get garbage collected
                    questionChoosers[i] = LoggedDashboardChooser("$key/Question #${i + 1} Chooser")
                    val chooser = questionChoosers[i]
                    questions[i].responses.keys.forEach { option ->
                        chooser.addOption(
                            option, option
                        )
                    }
                } else {
                    questionPublishers[i].set("")
                    questionChoosers[i] = LoggedDashboardChooser("$key/Question #${i + 1} Chooser")
                    questionChoosers[i].addDefaultOption("N/A", "N/A")
                }
            }
        }

        // Update the routine and responses
        lastRoutine = selectedRoutine
        lastResponses = mutableMapOf()
        lastRoutine.questions.indices.forEach { i ->
            val responseString = questionChoosers[i].get()
            lastResponses[responseString] =
                (if (responseString == null) lastRoutine.questions[i].responses.values.first()
                else lastRoutine.questions[i].responses[responseString])!!
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
