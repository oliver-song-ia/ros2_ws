from pxr import UsdPhysics, PhysxSchema, UsdGeom, Gf
import omni.usd
import time
from omni.isaac.core.utils.stage import add_reference_to_stage
def get_all_children(prim):
    children = []
    def recurse(p):
        for child in p.GetChildren():
            children.append(child)
            recurse(child)
    recurse(prim)
    return children

def main():

    #########################################
    # TODO: change the path to your glb file
    glb_path = "/home/hanxi/code/ia_robot_sim/src/ia_robot_sim/glb/house_00000.glb"
    prim_path = "/World/house"
    ########################################

    stage = omni.usd.get_context().get_stage()

    
    add_reference_to_stage(glb_path, prim_path)

    time.sleep(1.0)

    root_prim = stage.GetPrimAtPath(prim_path)
    if not root_prim or not root_prim.IsValid():
        print(f"ERROR: Prim at {prim_path} not found or invalid!")
        return

    # Apply +90 degree rotation on X axis to convert from Y-up to Z-up
    xformable = UsdGeom.Xformable(root_prim)
    xformable.ClearXformOpOrder()
    xformable.AddRotateXOp().Set(90.0)

    for prim in get_all_children(root_prim):
        if prim.GetTypeName() == "Xform" and prim.IsInstance():
            UsdPhysics.CollisionAPI.Apply(prim)
            PhysxSchema.PhysxCollisionAPI.Apply(prim)
            print(f"Collision enabled for instanced Xform: {prim.GetPath()}")
        if prim.GetTypeName() == "Mesh":
            UsdPhysics.CollisionAPI.Apply(prim)
            PhysxSchema.PhysxCollisionAPI.Apply(prim)
            print(f"Collision enabled for: {prim.GetPath()}")

if __name__ == "__main__":
    main()
