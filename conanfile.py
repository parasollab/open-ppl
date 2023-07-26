from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout
from conan.tools.build import check_max_cppstd, check_min_cppstd


class PPLRecipe(ConanFile):
    name = "ppl"
    version = "1.1.0"

    # Optional metadata
    license = "BSD 3-Clause License"
    author = "Parasol"
    url = "https://parasollab.web.illinois.edu/resources/"
    description = "Parasol Planning Library"
    topics = ("Motion Planning", "Robotics")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": False}

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "CMakeLists.txt", "src/*", "include/*"

    generators = "CMakeDeps"

    def build_requirements(self):
        self.tool_requires("cmake/[>=3.20.0]")

    def validate(self):
        check_min_cppstd(self, "17")

    def requirements(self):
        self.requires("eigen/3.4.0", override=True)
        self.requires("bullet3/3.24")
        self.requires("boost/1.82.0")
        self.requires("mpfr/4.1.0")
        self.requires("gmp/6.2.1")
        self.requires("nlohmann_json/3.10.5")
        self.requires("tinyxml/2.6.2")
        self.requires("catch2/3.3.2")
        self.requires("cgal/5.5.2")

        self.requires("qt/6.5.1")
        self.requires("opencv/4.5.5")

        #self.requires("dlib/19.24")

        # dependencies for above packages (explicit multi-version conflict resolution)
        self.requires("libjpeg/9e")
        self.requires("libpng/1.6.39")
        self.requires("libwebp/1.3.0", override=True)
        self.requires("sqlite3/3.41.1")
        self.requires("ffmpeg/4.4.3", override=True)
        self.requires("xz_utils/5.4.2", override=True)

        self.requires("glib/2.77.0", override=True)

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.definitions["CMAKE_POSITION_INDEPENDENT_CODE"] = self.options.fPIC
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = ["ppl_library", "ppl_mp_library"]
