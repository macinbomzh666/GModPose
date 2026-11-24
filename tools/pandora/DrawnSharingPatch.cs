using HKX2E;
using Pandora.API.Patch;
using Pandora.API.Patch.Skyrim64;

namespace GModPose.Pandora;

public sealed class DrawnSharingPatch : ISkyrim64Patch
{
    public RuntimeMode Mode => RuntimeMode.Serial;
    public RunOrder Order => RunOrder.PostLaunch;

    public void Run(IProjectManager projects)
    {
        if (projects.TryActivatePackFilePriority("0_master", out var file) && file is IPackFileGraph graph)
            Apply(graph);
    }

    public static bool Apply(IPackFileGraph graph)
    {
        var revision = graph.StringData.variableNames.IndexOf("GModPoseShareRevision");
        if (revision < 0 || revision >= graph.VariableValueSet.wordVariableValues.Count)
            return false;
        if (graph.VariableValueSet.wordVariableValues[revision].value >= 2)
            return true;

        var native = graph.BinaryDeserializer.Context.ReferenceIDMap.Values.OfType<hkbStateMachine>()
            .FirstOrDefault(machine => machine.name == "Default_Behavior");
        var weapons = native?.states.FirstOrDefault(state => state.name == "Weap_Readied_State");
        var magic = native?.states.FirstOrDefault(state => state.name == "Magic_Behavior_State");
        if (weapons?.generator is not hkbGenerator weaponGenerator || magic?.generator is not hkbGenerator magicGenerator)
            return false;

        const string startEvent = "GModPoseShareDrawnStart";
        const string stopEvent = "GModPoseShareDrawnStop";
        var start = graph.FindEvent(startEvent);
        var stop = graph.FindEvent(stopEvent);
        if (start < 0) start = graph.AddDefaultEvent(startEvent);
        if (stop < 0) stop = graph.AddDefaultEvent(stopEvent);
        if (start < 0 || stop < 0) return false;

        Wrap(weapons, weaponGenerator, start, stop);
        Wrap(magic, magicGenerator, start, stop);
        graph.VariableValueSet.wordVariableValues[revision] = new hkbVariableValue { value = 2 };
        return true;
    }

    private static void Wrap(hkbStateMachineStateInfo stance, hkbGenerator generator, int start, int stop)
    {
        // Both states use the native generator. Neither transition equips a
        // weapon or enters the unarmed idle graph.
        var wrapper = hkbStateMachine.GetDefault();
        // PlayerHub sees the default state as vanilla and the sharing state as modded.
        wrapper.name = "Default_Behavior";
        wrapper.startStateId = 0;
        wrapper.states = [State(stance.name, generator, 0, start, 1),
                          State("GModPoseShareDrawn", generator, 1, stop, 0)];
        stance.generator = wrapper;
    }

    private static hkbStateMachineStateInfo State(string name, hkbGenerator generator, int stateId, int eventId, int target)
    {
        var transition = hkbStateMachineTransitionInfo.GetDefault();
        transition.eventId = eventId;
        transition.toStateId = target;
        var transitions = hkbStateMachineTransitionInfoArray.GetDefault();
        transitions.transitions = [transition];
        var state = hkbStateMachineStateInfo.GetDefault();
        state.name = name;
        state.generator = generator;
        state.stateId = stateId;
        state.enable = true;
        state.probability = 1;
        state.transitions = transitions;
        return state;
    }
}
