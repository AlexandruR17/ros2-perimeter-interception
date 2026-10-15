# State of practice: robotic perimeter defense

Background research for the project. It summarises public, mostly official
documents on how military installations are protected with sensors and
unmanned systems in the United States and in Ukraine, the policy framework
for autonomy in the use of force, and known weaknesses of the middleware
the project relies on. Each finding is mapped to a design decision of the
project at the end.

The base used in the simulation (FOB-3) is fictional and is not modelled
on, nor affiliated with, any of the organisations cited below. Those
organisations appear only as sources.

Method and caveats:

- Only open sources are used; nothing here is restricted information.
- Official publications (government sites, doctrine, standards, UN and
  ICRC documents) are preferred. Press and think-tank sources are used for
  field practice and are marked as such.
- Figures reported by a single source are flagged. Numbers about the war
  in Ukraine change quickly; re-check them against the original release
  before quoting them.
- Sources were accessed in October 2026.

## 1. United States

### 1.1 Doctrine and standards for installation security

| Document | What it says that matters here |
|---|---|
| Joint Publication 3-10, *Joint Security Operations in Theater* (2019) [1] | Base defense doctrine. Since 2006 it consolidates the former JP 3-10.1 *Joint Tactics, Techniques, and Procedures for Base Defense*; it covers the protection of bases and lines of communication, the layered defense of a base and the threat levels a base must be prepared for. |
| UFC 4-022-03, *Security Fences and Gates* (2013, active) [2] | Design, selection and installation of security fences and gates, with standard drawings. It explicitly leaves threat-level determination and security operations to other documents, i.e. the fence only delays an intruder: detection and response must be provided by something else. |

### 1.2 Robotic and integrated base-security programs

- **MDARS (Mobile Detection Assessment Response System)** [3][4]: a joint
  Army/Navy program for unarmed autonomous security robots at storage
  sites and depots. The robots patrol, check for intruders, investigate
  alarm sources and assess barriers; one operator supervises many
  vehicles and human responders handle the breach. Reported uses include
  the Nevada National Security Site and Marine Corps evaluations for air
  base ground defense (2014). Most sources date from 2000-2016; current
  fielding status was not confirmed.
- **Quadruped UGVs at Tyndall Air Force Base** (from 2020) [5][6]: four
  semi-autonomous legged robots supplied to the base security forces
  squadron for surveillance and perimeter patrol. They follow programmed
  patrol routes, stream sensor data to the operations centre, and an
  operator can take over and talk through the robot.
- **Integrated Base Defense Security System (IBDSS)** [7]: the Air Force
  family of systems that lets security forces "detect, assess and track
  hostile forces": fixed-site electronic security, deployable tactical
  sensors and command and control, and delay/denial systems. A follow-on
  contract (GUARD, draft request for proposals September 2026) asks for a
  modular, open architecture.

### 1.3 Counter-drone and autonomy initiatives

- **Counter-small UAS strategy** (January 2021, public) [8] and
  **Strategy for Countering Unmanned Systems** (December 2024, classified;
  only press summaries) [9]: small drones are named the most significant
  threat to installations, including in the homeland.
- **Replicator** (announced August 2023) [10]: fielding attritable
  autonomous systems at scale; its second line of effort (2024) is
  countering small UAS at critical installations.

### 1.4 Policy on autonomy

- **DoD Directive 3000.09, *Autonomy in Weapon Systems*** (25 January 2023,
  reissues the 2012 version) [11]: autonomous and semi-autonomous weapon
  systems must allow commanders and operators to exercise appropriate
  levels of human judgment over the use of force, and must pass rigorous
  verification, validation and realistic testing. Critics note that it
  does not require a human in the loop for every engagement.
- **DoD AI Ethical Principles** (24 February 2020) [12]: responsible,
  equitable, traceable, reliable, governable. "Governable" includes the
  ability to detect unintended behaviour and to disengage or deactivate.
- **Political Declaration on Responsible Military Use of AI and Autonomy**
  (U.S. Department of State, 2023) [13]: non-binding; endorsed by about 50
  states by late 2023. Calls for compliance with international
  humanitarian law, testing, auditability and a deactivation capability.

## 2. Ukraine

- **A dedicated branch for unmanned systems.** The Unmanned Systems Forces
  were created as a separate branch of the Armed Forces in June 2024
  (presidential decree following a February 2024 decision), with a remit
  covering air, sea and ground robotic systems [14][15]. Sources disagree
  on the exact decree number and dates.
- **Scale of ground robots.** According to the Ministry of Defence, the
  number of ground robotic systems codified for operational use grew from
  13 (2023) to about 60 (2024) and 40 by August 2025, almost all of them
  domestically made [16][17]. A procurement official announced a plan for
  15,000 ground robotic systems at the front in 2025 [18]; later MoD
  releases report larger 2026 contracts (figures differ between releases).
- **What the ground robots actually do.** Mostly logistics (last-mile
  supply), casualty evacuation and engineering tasks, then remote weapon
  stations and one-way strike missions [16][18][19]. Documented cases
  include evacuations of wounded soldiers over 12-17 km under fire [19].
  The pressure behind this is that crewed evacuation near the front has
  become too dangerous.
- **Electronic warfare is the dominant constraint.** Radio links are
  jammed and terrain blocks line of sight; drone and ground-robot
  operators increasingly switch to fiber-optic control, which cannot be
  jammed but limits range and manoeuvre [20][21]. Being unjammable does not
  make a robot survivable: one press report quotes most ground robots as
  eventually lost to FPV drones (single source) [22].
- **Fast fielding.** Brave1, the government defense-tech cluster created
  in 2023 by several ministries and the General Staff, funds and tests
  new systems, including ground robots, on short cycles [23].

## 3. International framework

- **UN General Assembly resolution 78/241, *Lethal autonomous weapons
  systems*** (22 December 2023; 152 in favour, 4 against, 11 abstentions)
  [24]: international law, in particular international humanitarian law,
  applies to autonomous weapon systems; the role of humans in the use of
  force is a central question. Follow-up resolutions 79/62 (2024) and
  80/57 (2025).
- **ICRC position on autonomous weapon systems** (12 May 2021) [25]:
  recommends new legally binding rules, prohibiting unpredictable
  autonomous weapons and those designed to apply force against persons,
  and strictly regulating the others (limits on targets, duration,
  geographical scope, and requirements for human supervision and timely
  intervention and deactivation).

## 4. Security of the robot middleware

- ROS 2 communicates over DDS. In 2021 researchers reported 13
  vulnerabilities (12 across six DDS implementations, including Fast DDS
  and Cyclone DDS used by ROS 2, and one in the specification), presented
  at Black Hat Europe 2021 and registered as CVEs; most vendors have
  released patches [26][27]. A scan found hundreds of DDS services exposed
  on the public internet.
- The OMG DDS Security specification and its ROS 2 tooling (SROS2) add
  authentication, access control and encryption [28][29]. Their cost in
  latency and bandwidth, and their behaviour when the link degrades, is
  what the project will measure.

## 5. Implications for the project

| Finding | Design decision | Where |
|---|---|---|
| A fence only delays; detection and response must be layered (JP 3-10, UFC 4-022-03) | Fence sensor + lidar tracking + mobile interceptors; fence alerts are treated as cues, not as tracks | M1-M2 |
| Security robots patrol and investigate alarms, one operator supervises many (MDARS, Tyndall) | Whole-base patrol route, alert investigation, C2 dashboard with a single operator | M2, dashboard |
| Small drones are the main threat to installations (DoD C-UAS strategies, Replicator 2) | The threat model includes drones besides people and ground robots | A, F |
| Human judgment over the use of force (DoDD 3000.09, ICRC, UNGA 78/241) | No lethal action against persons: robots capture or block, a human acknowledges every capture and detention | M2-M4 |
| Ground robots are mostly used for logistics and evacuation (Ukraine) | A dedicated transport robot escorts captured intruders, separate from the interceptors | M4 |
| Radio links are jammed; fiber is not always possible (Ukraine) | Simulated network with jamming; degraded modes where robots keep a safe behaviour without C2 | C |
| Robots must recognise friendly robots without communications | Robot-to-robot recognition independent of the radio link | C |
| DDS has had real vulnerabilities; security has a cost | SROS2 enabled and measured; an attack test on the unsecured configuration | D, M7 |
| Governable systems must be testable and auditable (AI Ethical Principles, Political Declaration) | Exercise events logged on a latched topic, repeatable scenarios, comparison of algorithms | M7 |

## References

Official and primary sources are marked **[O]**; press, think-tank and
secondary sources **[S]**.

1. **[O]** Joint Chiefs of Staff, *JP 3-10 Joint Security Operations in Theater*, 25 July 2019. https://www.jcs.mil/Portals/36/Documents/Doctrine/pubs/jp3_10.pdf (copy: https://irp.fas.org/doddir/dod/jp3_10.pdf)
2. **[O]** Department of Defense, *UFC 4-022-03 Security Fences and Gates*, 1 October 2013. https://www.wbdg.org/dod/ufc/ufc-4-022-03
3. **[O]** MDARS technical papers, Defense Technical Information Center. https://apps.dtic.mil/sti/pdfs/ADA422019.pdf , https://apps.dtic.mil/sti/pdfs/ADA422496.pdf
4. **[O]** U.S. Marine Corps, "Combat Center Marines first look at unmanned security vehicle". https://www.29palms.marines.mil/Articles/Article/499615/combat-center-marines-first-look-at-unmanned-security-vehicle
5. **[O]** U.S. Air Force, "Hanscom AFB teams support testing of AI and 'robotic dogs' at Tyndall AFB". https://www.af.mil/News/Article-Display/Article/2601270/hanscom-afb-teams-support-testing-of-ai-and-robotic-dogs-at-tyndall-afb/
6. **[S]** ASIS Security Management, "Robot Dogs Set to Augment U.S. Air Force Base Security", December 2020. https://www.asisonline.org/security-management-magazine/latest-news/today-in-security/2020/december/Robot-Dogs-Set-to-Augment-Air-Force-Base-Security/
7. **[S]** Washington Technology, "Air Force unveils first draft of $2B base security tech recompete", September 2026. https://www.washingtontechnology.com/contracts/2026/09/air-force-unveils-first-draft-2b-base-security-tech-recompete/416270/
8. **[O]** Department of Defense, *Counter Small Unmanned Aircraft Systems Strategy*, January 2021. https://media.defense.gov/2021/Jan/07/2002561080/-1/-1/0/DEPARTMENT-OF-DEFENSE-COUNTER-SMALL-UNMANNED-AIRCRAFT-SYSTEMS-STRATEGY.pdf
9. **[S]** Federal News Network, "Pentagon releases classified strategy for countering drones", December 2024. https://federalnewsnetwork.com/?p=5254290
10. **[O]** Defense Innovation Unit, "Implementing the Department of Defense Replicator Initiative". https://www.diu.mil/latest/implementing-the-department-of-defense-replicator-initiative-to-accelerate ; **[O]** Congressional Research Service, *DOD Replicator Initiative: Background and Issues for Congress* (IF12611). https://www.everycrsreport.com/files/2025-09-19_IF12611_85d2a87c9f2339f5b200dcab7c98855bd8a8a7e9.html
11. **[O]** Department of Defense, *DoD Directive 3000.09 Autonomy in Weapon Systems*, 25 January 2023. https://www.esd.whs.mil/Portals/54/Documents/DD/issuances/dodd/300009p.PDF
12. **[O]** U.S. Army, "DOD adopts 5 principles of artificial intelligence ethics", 2020. https://www.army.mil/article/233690
13. **[O]** U.S. Department of State, *Political Declaration on Responsible Military Use of Artificial Intelligence and Autonomy*, 2023; endorsement summary: https://www.jbsa.mil/News/News/Article/3597476/us-endorses-responsible-ai-measures-for-global-militaries/
14. **[S]** Janes, "Ukraine establishes world's first unmanned force", June 2024. https://janes.com/osint-insights/defence-news/air/ukraine-conflict-ukraine-establishes-worlds-first-unmanned-force
15. **[S]** LB.ua, "Zelenskyy approves NSDC decision", 26 June 2024. https://en.lb.ua/news/2024/06/26/30238_zelenskyy_approves_nsdc_decision.html
16. **[O]** Ministry of Defence of Ukraine, "In July, 8 new ground robotic systems were authorized for operational use", August 2025. https://mod.gov.ua/en/news/in-july-8-new-ground-robotic-systems-were-authorized-for-operational-use
17. **[O]** Ministry of Defence of Ukraine, "Nearly thirty ground robotic systems approved for operational use since the beginning of the year", 2025. https://mod.gov.ua/en/news/nearly-thirty-ground-robotic-systems-approved-for-operational-use-since-the-beginning-of-the-year
18. **[S]** The Jamestown Foundation, "Ukraine Becomes World Leader in Unmanned Ground Vehicles". https://jamestown.org/ukraine-becomes-world-leader-in-unmanned-ground-vehicles/
19. **[S]** ASPI The Strategist, "Ground robots are transforming battle-casualty evacuation in Ukraine", 2025. https://www.aspistrategist.org.au/ground-robots-are-transforming-battle-casualty-evacuation-in-ukraine/
20. **[S]** CSIS, "The Russia-Ukraine Drone War: Innovation on the Frontlines and Beyond". https://www.csis.org/analysis/russia-ukraine-drone-war-innovation-frontlines-and-beyond
21. **[S]** The War Zone, "Unmanned Ground Vehicles Controlled Via Fiber Optic Cables Being Tested By Ukraine". https://www.twz.com/land/unmanned-ground-vehicles-controlled-via-fiber-optic-cables-being-tested-by-ukraine
22. **[S]** Forbes, D. Kirichenko, "A Ukrainian ground robot drove into a Russian-held building and blew it up", 13 May 2026. https://www.forbes.com/sites/davidkirichenko/2026/05/13/a-ukrainian-ground-robot-drove-into-a-russian-held-building-and-blew-it-up/
23. **[O]** Cabinet of Ministers of Ukraine, "Ukraine launches BRAVE1 defence tech cluster", April 2023. https://www.kmu.gov.ua/en/news/v-ukraini-zapustyly-defense-tech-cluster-brave1-iakyi-stymuliuvatyme-rozvytok-viiskovykh-innovatsii-ta-oboronnykh-tekhnolohii ; https://brave1.gov.ua/en
24. **[O]** United Nations General Assembly, *A/RES/78/241 Lethal autonomous weapons systems*, 22 December 2023. https://digitallibrary.un.org/record/4033027
25. **[O]** International Committee of the Red Cross, *ICRC position on autonomous weapon systems*, 12 May 2021. https://www.icrc.org/en/document/icrc-position-autonomous-weapon-systems
26. **[S]** Canonical, "Security vulnerabilities on the Data Distribution Service (DDS)". https://ubuntu.com/blog/security-vulnerabilities-on-the-data-distribution-service-dds
27. **[S]** Trend Micro Research, "Defending the Supply Chain: Why DDS is Critical in Industrial and Software Systems", 2022. https://www.trendmicro.com/en_us/research/22/a/defending-the-supply-chain-why-dds-is-critical-in-industrial-and-software-systems.html
28. **[O]** Object Management Group, *DDS Security* specification, version 1.1. https://www.omg.org/spec/DDS-SECURITY/
29. **[S]** V. Mayoral-Vilches, R. White, G. Caiazza, M. Arguedas, "SROS2: Usable Cyber Security Tools for ROS 2", IROS 2022.
