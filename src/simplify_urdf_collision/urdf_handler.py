from urdf_parser_py.urdf import URDF, Mesh
from .color import COLORS


class URDFHandler:
    def __init__(self, filename, excluded_links):
        self.robot = URDF.from_xml_file(filename)
        links = self.robot.link_map
        self.collision_models = {}
        for link_name, link in links.items():
            if link_name in excluded_links:
                continue
            c_single_link = []
            for c in link.collisions:
                if isinstance(c.geometry, Mesh):
                    c_single_link.append(c)
            if c_single_link:
                self.collision_models[link_name] = c_single_link

        total = sum(len(c) for c in self.collision_models.values())
        print(
            f"{COLORS.OKGREEN}{total} mesh collision models to be simplified "
            f"were found in the URDF (excluded links excluded){COLORS.ENDC}"
        )

    def get_filenames(self, interactive):
        """Returns a dict of link_name: [collision_model_1, collision_model_2, ...].

        If interactive is enabled, uses command-line input to deselect models.
        """
        if interactive:
            while True:
                links = list(self.collision_models.keys())
                for i, l in enumerate(links):
                    print(f"{COLORS.OKBLUE}[{i}]{COLORS.ENDC} {l} ({len(self.collision_models[l])})")
                selected_l = input(
                    f"Enter {COLORS.OKBLUE}e{COLORS.ENDC} to exit or link number to "
                    f"deselect collision models {COLORS.OKBLUE}[0]...[{len(links)-1}]{COLORS.ENDC}:"
                )
                if selected_l == "e":
                    break
                else:
                    selected_l_int = int(selected_l)
                    while True:
                        if len(self.collision_models[links[selected_l_int]]) == 0:
                            self.collision_models.pop(links[selected_l_int])
                            break

                        for j, c in enumerate(self.collision_models[links[selected_l_int]]):
                            print(f"{COLORS.OKBLUE}[{j}]{COLORS.ENDC} {c.geometry.filename}")
                        selected_c = input(
                            f"Enter e to exit or collision model number to deselect "
                            f"{COLORS.OKBLUE}[0]...[{len(self.collision_models[links[selected_l_int]])-1}]{COLORS.ENDC}:"
                        )
                        if selected_c == "e":
                            break
                        else:
                            selected_c_int = int(selected_c)
                            del self.collision_models[links[selected_l_int]][selected_c_int]
        return self.collision_models

    def write_urdf(self, filename):
        s = self.robot.to_xml_string()
        with open(filename, "w") as f:
            f.write(s)
