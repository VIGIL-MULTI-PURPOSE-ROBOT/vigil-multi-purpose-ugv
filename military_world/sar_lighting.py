# sar_lighting.py -- switch lighting presets inside Blender.
# Paste into the Scripting workspace, or Text > Run Script.
import bpy, json, math

def apply(preset):
    found = False
    for o in bpy.data.objects:
        if o.get("sar_object") == "LIGHT" and o.get("lighting_preset"):
            tags = [t.strip() for t in str(o["lighting_preset"]).split(",")]
            on = preset in tags
            o.hide_viewport = not on
            o.hide_render = not on
            if on and o.get("world_color"):
                col = json.loads(o["world_color"])
                w = bpy.context.scene.world
                nt = w.node_tree
                for n in nt.nodes:
                    if n.type == 'BACKGROUND':
                        n.inputs[0].default_value = (col[0], col[1], col[2], 1.0)
                        n.inputs[1].default_value = float(o["world_strength"])
                    if n.type == 'VOLUME_SCATTER':
                        n.inputs["Density"].default_value = float(o["fog_density"])
                w["active_preset"] = preset
                found = True
    print(("applied " if found else "no sun found for ") + preset)

# apply("DAY") / "CLOUDY" / "SUNSET" / "LOW_LIGHT" / "NIGHT" / "FOG"
apply("DAY")
