from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake, cmake_layout
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
    default_options = {"shared": False, "fPIC": True}

    generators = "CMakeToolchain", "CMakeDeps"

    def build_requirements(self):
        self.tool_requires("cmake/[>=3.20.0]")

    def validate(self):
        check_min_cppstd(self, "17")

    def requirements(self):
        self.requires("cgal/5.6.1")
        self.requires("boost/1.83.0")
        self.requires("eigen/3.4.0")
        self.requires("bullet3/3.25")
        self.requires("mpfr/4.2.1")
        self.requires("gmp/6.3.0")
        self.requires("nlohmann_json/3.10.5")
        self.requires("tinyxml2/9.0.0")
        self.requires("catch2/3.3.2")
        self.requires("qt/6.3.2")

        # dependencies for above packages (explicit multi-version conflict resolution)
        self.requires("libpng/1.6.40", override=True)
        self.requires("glib/2.77.0", override=True)

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

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