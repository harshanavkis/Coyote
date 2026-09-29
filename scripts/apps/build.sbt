val spinalVersion = "1.14.2"

lazy val root = (project in file("."))
  .settings(
    name         := "coyote-spinal-kernel",
    version      := "1.0",
    scalaVersion := "2.13.18",

    libraryDependencies ++= Seq(
      "com.github.spinalhdl" %% "spinalhdl-core" % spinalVersion,
      "com.github.spinalhdl" %% "spinalhdl-lib"  % spinalVersion,
      compilerPlugin("com.github.spinalhdl" %% "spinalhdl-idsl-plugin" % spinalVersion)
    ),

    Compile / run / fork := true
  )
