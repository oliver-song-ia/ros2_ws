"""
XML Behavior Tree Definition

This file defines the behavior tree structure in XML format.
It can be used with py_trees or Nav2's BT navigator.

Note: The current implementation builds the tree programmatically,
but this XML can be used as a reference or for visualization.
"""

behavior_tree_xml = """
<root main_tree_to_execute="MissionTree">
    <BehaviorTree ID="MissionTree">
        <Sequence name="MissionSequence">
            <!-- Wait for mission trigger -->
            <Action ID="WaitForTrigger" name="Wait for mission trigger"/>
            
            <!-- Check prerequisites -->
            <Condition ID="IsHumanPoseAvailable" name="Check human pose available"/>
            
            <!-- Navigation phase with retry -->
            <Timeout seconds="120">
                <RetryUntilSuccessful num_attempts="3">
                    <Action ID="NavigateToHuman" name="Navigate to human location"/>
                </RetryUntilSuccessful>
            </Timeout>
            
            <!-- Manipulation phase -->
            <Sequence name="ManipulationPhase">
                <Action ID="StartManipulation" name="Start manipulation processes"/>
                <Condition ID="IsManipulationComplete" name="Wait for manipulation complete"/>
                <Action ID="StopManipulation" name="Stop manipulation processes"/>
            </Sequence>
        </Sequence>
    </BehaviorTree>
</root>
"""
