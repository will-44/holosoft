# holosoft
an old stm32 navigation firmeware for a omnirobot

```mermaid
classDiagram
    class main {
    }
    class brain {
    }
    class secretary {
    }
    class it {
    }
    class joystick {
    }
    class pilote {
    }
    class odometry {
    }
    class corrector {
    }
    class motors {
    }

    main --> brain
    brain --> secretary
    it --> secretary
    it --> joystick
    secretary --> pilote
    pilote --> corrector
    pilote --> odometry
    joystick --> pilote
    corrector --> motors


```
