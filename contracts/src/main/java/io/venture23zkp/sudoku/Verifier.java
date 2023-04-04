package io.venture23zkp.sudoku;

import score.Context;
import score.annotation.External;

import java.math.BigInteger;

import java.util.Arrays;

public class Verifier {
    static class ByteUtil {
        public static final byte[] EMPTY_BYTE_ARRAY = new byte[0];
        public static final byte[] ZERO_BYTE_ARRAY = new byte[]{0};

        public static int firstNonZeroByte(byte[] data) {
            for (int i = 0; i < data.length; ++i) {
                if (data[i] != 0) {
                    return i;
                }
            }
            return -1;
        }

        public static byte[] stripLeadingZeroes(byte[] data) {

            if (data == null)
                return null;

            final int firstNonZero = firstNonZeroByte(data);
            switch (firstNonZero) {
                case -1:
                    return ZERO_BYTE_ARRAY;

                case 0:
                    return data;

                default:
                    byte[] result = new byte[data.length - firstNonZero];
                    System.arraycopy(data, firstNonZero, result, 0, data.length - firstNonZero);

                    return result;
            }
        }

        static byte[] encodeInto64Bytes(byte[] w1, byte[] w2) {
            byte[] res = new byte[64];

            w1 = stripLeadingZeroes(w1);
            w2 = stripLeadingZeroes(w2);

            System.arraycopy(w1, 0, res, 32 - w1.length, w1.length);
            System.arraycopy(w2, 0, res, 64 - w2.length, w2.length);

            return res;
        }

        static byte[] encodeInto32Bytes(byte[] w) {
            byte[] res = new byte[32];
            w = stripLeadingZeroes(w);
            System.arraycopy(w, 0, res, 32 - w.length, w.length);
            return res;
        }


        static byte[] concat(byte[] w1, byte[] w2) {
            byte[] res = new byte[w1.length+w2.length];
            System.arraycopy(w1, 0, res, 0, w1.length);
            System.arraycopy(w2, 0, res, w1.length, w2.length);
            return res;
        }
    }

    static class Pairing {

        static class Scalar {
            public BigInteger v;
            Scalar(BigInteger v) {
                this.v = v;
            }
            public static Scalar decode(byte[] r) {
                Context.require(r.length == 32, "Scalar: a scalar should be 32 bytes");
                return new Scalar(new BigInteger(r));
            }
            public byte[] encode() {
                return ByteUtil.encodeInto32Bytes(v.toByteArray());
            }
        }

        static class G1Point {
            public BigInteger x;
            public BigInteger y;
            G1Point(BigInteger x, BigInteger y) {
                this.x = x;
                this.y = y;
            }
            public static G1Point decode(byte[] r) {
                Context.require(r.length == 64, "G1Point: a G1Point should be 64 bytes");
                return new G1Point(
                        new BigInteger(Arrays.copyOfRange(r, 0, 32)),
                        new BigInteger(Arrays.copyOfRange(r, 32, 64)));
            }
            public byte[] encode() {
                return ByteUtil.encodeInto64Bytes(x.toByteArray(), y.toByteArray());
            }
        }

        static class G2Point {
            public BigInteger xi;
            public BigInteger xr;
            public BigInteger yi;
            public BigInteger yr;
            G2Point(BigInteger xi, BigInteger xr, BigInteger yi, BigInteger yr) {
                this.xi = xi;
                this.xr = xr;
                this.yi = yi;
                this.yr = yr;
            }
            public byte[] encode() {
                byte[] x = ByteUtil.encodeInto64Bytes(xi.toByteArray(), xr.toByteArray());
                byte[] y = ByteUtil.encodeInto64Bytes(yi.toByteArray(), yr.toByteArray());
                return ByteUtil.concat(x, y);
            }
        }

        public G1Point P1() {
            return new G1Point(
                    new BigInteger("1"),
                    new BigInteger("2")
            );
        }

        public static G1Point negate(G1Point p) {
            // The prime q in the base field F_q for G1
            BigInteger q = new BigInteger("21888242871839275222246405745257275088696311157297823662689037894645226208583");
            if (p.x.equals(BigInteger.ZERO) && p.y.equals(BigInteger.ZERO)) {
                return new G1Point(
                        new BigInteger("0"),
                        new BigInteger("0")
                );
            }
            return new G1Point(
                    p.x,
                    q.subtract(p.y.mod(q))
            );
        }

        public G2Point P2() {

            // Original code point
            return new G2Point(
                    new BigInteger("11559732032986387107991004021392285783925812861821192530917403151452391805634"),
                    new BigInteger("10857046999023057135944570762232829481370756359578518086990519993285655852781"),
                    new BigInteger("4082367875863433681332203403145435568316851327593401208105741076214120093531"),
                    new BigInteger("8495653923123431417604973247489272438418190587263600148770280649306958101930")
                );
        }

        public static G1Point addition(G1Point p1, G1Point p2) {
            byte[] res = Context.altBN128("add", ByteUtil.concat(p1.encode(), p2.encode()));
            G1Point p = G1Point.decode(res);
            return p;
        }

        public static G1Point scalar_mul(G1Point p, Scalar s) {
            byte[] res;
            res = Context.altBN128("mul", ByteUtil.concat(p.encode(), s.encode()));
            return G1Point.decode(res);
        }

        public static boolean pairing(G1Point[] p1, G2Point[] p2) {
            Context.require(p1.length == p2.length, "Pairing: G1 and G2 points must have same length");
            Context.require(p1.length > 0, "Paring: Must have some points");

            byte[] arg = ByteUtil.concat(p1[0].encode(), p2[0].encode());
            for (int i=1; i<p1.length; i++) {
                byte[] tmp = ByteUtil.concat(p1[i].encode(), p2[i].encode());
                arg = ByteUtil.concat(arg, tmp);
            }

            byte[] res = Context.altBN128("pairing", arg);

            return !Scalar.decode(res).v.equals(BigInteger.ZERO);
        }

        public boolean pairingProd2(G1Point a1, G2Point a2, G1Point b1, G2Point b2) {
            G1Point[] p1 = new G1Point[]{a1, b1};
            G2Point[] p2 = new G2Point[]{a2, b2};
            return pairing(p1, p2);
        }

        public boolean pairingProd3(G1Point a1, G2Point a2, G1Point b1, G2Point b2, G1Point c1, G2Point c2) {
            G1Point[] p1 = new G1Point[]{a1, b1, c1};
            G2Point[] p2 = new G2Point[]{a2, b2, c2};
            return pairing(p1, p2);
        }

        public static boolean pairingProd4(G1Point a1, G2Point a2, G1Point b1, G2Point b2, G1Point c1, G2Point c2, G1Point d1, G2Point d2) {
            G1Point[] p1 = new G1Point[]{a1, b1, c1, d1};
            G2Point[] p2 = new G2Point[]{a2, b2, c2, d2};
            return pairing(p1, p2);
        }

    }

    static class VerifyingKey {
        Pairing.G1Point alfa1;
        Pairing.G2Point beta2;
        Pairing.G2Point gamma2;
        Pairing.G2Point delta2;

        Pairing.G1Point[] IC;
    }

    static class Proof {
        Pairing.G1Point A;
        Pairing.G2Point B;
        Pairing.G1Point C;

    }

    public VerifyingKey verifyingKey () {
        VerifyingKey vk = new VerifyingKey();
        vk.alfa1 = new Pairing.G1Point(
                new BigInteger("20491192805390485299153009773594534940189261866228447918068658471970481763042"),
                new BigInteger("9383485363053290200918347156157836566562967994039712273449902621266178545958")
        );

        vk.beta2 = new Pairing.G2Point(
                    new BigInteger("4252822878758300859123897981450591353533073413197771768651442665752259397132"),
                    new BigInteger("6375614351688725206403948262868962793625744043794305715222011528459656738731"),
                    new BigInteger("21847035105528745403288232691147584728191162732299865338377159692350059136679"),
                    new BigInteger("10505242626370262277552901082094356697409835680220590971873171140371331206856")
                );

        vk.gamma2 = new Pairing.G2Point(
                    new BigInteger("11559732032986387107991004021392285783925812861821192530917403151452391805634"),
                    new BigInteger("10857046999023057135944570762232829481370756359578518086990519993285655852781"),
                    new BigInteger("4082367875863433681332203403145435568316851327593401208105741076214120093531"),
                    new BigInteger("8495653923123431417604973247489272438418190587263600148770280649306958101930")
                );

        vk.delta2 = new Pairing.G2Point(
                    new BigInteger("12604013977472581151659936597642752764077924661869116760551819948903890036993"),
                    new BigInteger("4403512442465658369171257027301292044536470187728846204174295365930064264252"),
                    new BigInteger("14735665949494264117597607149374293287239539255928185820113321739426730720343"),
                    new BigInteger("3374690572333174428369126592848775563077396800405608230848449530824067426958")
                );

        vk.IC = new Pairing.G1Point[] {
            
            new Pairing.G1Point( 
                new BigInteger("8289949299271585244411464387152706207855528662399497767604473248135248618501"),
                new BigInteger("602028220687630964399601756732824157515691527454261366150903802976420170833")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("14378614567796858531052006437535965614747837346393720460512563148291821987680"),
                new BigInteger("15555244453962110638848111886193360887199688654681164787526299320694262129098")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("5818922173783464046609914691700297183892869790347383162336881422857626965859"),
                new BigInteger("16733498363225660252327029519880970951571562997162023232022534229783200944238")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("5092993782537258807641439834601384333627245754665012352746699258865489532278"),
                new BigInteger("6302597501050031411235112864226972903773938518149854239673766462986427025444")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("4912632520132619583813816059928125354654448673050705078314256218971540126788"),
                new BigInteger("11170705717488484606670707756873148907411091666521545041257981292553708577566")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("1595281918557112211704460984484569663806970442108375730925678146147766726918"),
                new BigInteger("4002128832513127191510037546391710655871577304461341019521507937779207505917")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("4848878577437379474294890334689832184786197666705903504194333680434649524164"),
                new BigInteger("17656290862807488577688892178029519564873817612782446410171899189234418739783")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("15458421215813595686644631553118885907695001368264965790792731076399319844632"),
                new BigInteger("18175482125305858920189493969494979706909982663410096036861630371982741169954")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("16367691549466335040267024387442879586480972779448860888043445653702314255914"),
                new BigInteger("16964201102111217998081756184709321180201909045120686123033023610144146198147")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("3624796315693454340858424549157013728085407033440043076639306245580090456553"),
                new BigInteger("1976992820594523356909003405919050363801175539238311110789026921801806400573")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("17462752580625277929791051841334898397036652556807743384467723719471756123124"),
                new BigInteger("16819997995738403106739940618566093505702557494067025022199538185598311641889")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("9455335354694499719845134949859403501728654097679061286037970299476919687307"),
                new BigInteger("11433801080083265649202449339925741550582342053554550445021045871728610836418")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("711826627698756055311111679562196443188151692109575931219329561178330694567"),
                new BigInteger("18314649810316995488648964980733612463358535136926551238917599406285844382015")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("14608446715058089794085413469736036658914092070792544716950852890075274409731"),
                new BigInteger("2695090167984400054478654804756237615629139035154177928494046123382271249511")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("8486084525579340106086841388522353580839020657718292673085307633539044134395"),
                new BigInteger("6393293363501266802181709411693905416943772932733253886332274801069272857029")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("6202255976515487149767163339682948516114970756089708490698721090110492956927"),
                new BigInteger("11442965624243855528311888691934320347785169385055651312322980373665696393489")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("2427406078300016346099328251725341908151482030149106958359255522591825266495"),
                new BigInteger("6265007926905790788637060806705608471212554860237688560613474911422651815107")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("19933171165980578267191174519968941299991889312267300131836215856994898539018"),
                new BigInteger("20168475689498721001020662355446426060329614971207065705125179356502167710560")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("12689259411571922700677035916285013035656177480817156961917650557323344394345"),
                new BigInteger("5277825330739072678940889338712059795210076282120185727429469686265493502103")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("9572181380316732701213756245635389965201349057622438896085662205445374899183"),
                new BigInteger("15583315569515477306022406889191258635434676581474542717916819190864539062086")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("2847370930569562703234972432331706852935414642888275464108667473417416726831"),
                new BigInteger("10192328433568336163320751531103050503468245059547972583399278815624387305693")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("3148001468606845930653475266638748627250562000290912510211412705261566601429"),
                new BigInteger("12686526959694806548705987688908354287889083338681273821264937932619954056763")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("15280576648223251274763364258420219835509658816023079647749768436556238621747"),
                new BigInteger("4815190493976512290558059460845248554192947779234674915749071490141330696482")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("2465295209117577185256476161031150889678348434228998301944831595200267625788"),
                new BigInteger("20120231664591368310696951650461388658222378907697923067234400693415128277536")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("14926958798797811891313047859244214838984758020067646870774390125930012289483"),
                new BigInteger("7499490475698520742358190617911497604399108474351630797167563470294041713243")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("5651877378830823490627523597944668997982686830770791191445990804589882582113"),
                new BigInteger("20156092256424617643718039573665670734375679269028865644833703936500255606726")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("7504272470569355899868755870328617652241072213119181304548402945498570793373"),
                new BigInteger("8846469774373572483583795053024154332894980295714261652440255439028539531199")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("7745048837389693113391012407389872983446752414230852265947916122410104689796"),
                new BigInteger("2070490294080312503345152977264188593663505453301703963981438225029135972527")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("20099023108371251044091221805274920630779706150242970663140214657564551904454"),
                new BigInteger("17139272770902863131118168609572847350474393431405219174594382882785430177991")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("1617324759645918831739379720458663499051939261305008335559434794383369316548"),
                new BigInteger("4605675175146542492889736967530266081353823877324265448257028071244204406")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("11254693718672614707273259291499224653803930306190876730661415279095250333381"),
                new BigInteger("12997277227907748605385687012462781845302074152408687300767530601772431527185")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("12759837877965584982150839033126687246788298116170070303217409759549358540668"),
                new BigInteger("3229159766114859368462547836534815209726685360683152459382868759310053908643")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("17601099947681249733100541879949676517534774783229201706713729104290942219609"),
                new BigInteger("17586314489822200114649977200647626628785820437023085187047752307991516381875")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("3173808842118723795457418927018103399025554791377652540819112103959048160243"),
                new BigInteger("6263692657087894488172519024967769133961061579602483447526771231867588448409")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("14780580578852031295033129203578440340060013857681729906877751843744568193955"),
                new BigInteger("7960447707707366011985619839703261998516075554265661960452345826043779804995")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("10365742036653366417900680557726332048888260158303975951769167209046582841551"),
                new BigInteger("16456768020293053726376654363110050823855737375426068875727632682503546806876")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("14177320480300608460294641515436067381983953368356445509012139367828565767941"),
                new BigInteger("5431004102709619161359698922523286862058554813409364085671337768182736281939")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("293230444967840614758801127964754180733959229727216797122730917431359089589"),
                new BigInteger("14875590396326952645460931639321907919274963694110345592961084343819183764184")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("18797544178158770217059729669189426909978000361224522200793334540674917371567"),
                new BigInteger("20290877121459519702820410360738219352282672672374682507632565317959068205537")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("19958296981701316500551766209271460026883105709410468589166404883137323184604"),
                new BigInteger("12411592536332194574694527847089538174950924660173703277730359062249619955440")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("13702874537615381283113449753131332745396958885525462068226713415221846902504"),
                new BigInteger("466370650705419845197319871541863560025286121677183669802933529573193977261")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("1564722864213472680772898311763496009558833281684845399054028307501794337286"),
                new BigInteger("20241710348592242445200283866671461769301589136698889212494880082097108539801")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("15982456674004828717503956076306947057721908151371527963951232132385031872062"),
                new BigInteger("17613818844756238390196838244042616255770539035507001754056999389039571626841")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("21154988015533700698675720732333943098482824414601058289263181415678284495992"),
                new BigInteger("10766313902445190505121132727247301769415271192688035556652935692675690145385")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("20949636158745155667777739817736113880056425836755865081591721463582501686393"),
                new BigInteger("3517750746802880168063423739329892364089502864038479399503374331945072025181")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("16557957881893562520771397953038067039329409661728724076504826783253667009935"),
                new BigInteger("3493457307920328230834134382953710662116289888380744688650403718185028951054")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("12556176207794754796821244214452181104455244839246376329511829508923611065480"),
                new BigInteger("447949295467628042668399028550812979677407305895244103266757812138145370614")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("8358270255272926833865119696678885418069729347548951218154858098536224018586"),
                new BigInteger("4740889545402143211940199025090609186228064185298579298606994108445843952341")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("5661867920706466694045495365863083535069105832260025514628509763613650455129"),
                new BigInteger("631278856580370108159533506912607278402808979356091896671743635451515381272")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("958534180547026515252870767579970700576016759464081791075393420254385766962"),
                new BigInteger("21598483303556483555587470445152221278689809423938538591034593092855545113630")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("9059118511341741927452873156535905481277004674790531969589878848965634995721"),
                new BigInteger("7759076015762461843955064400708628689665128176518433370692126258316781044589")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("18000189169591715341879655441866730617951284169166435407534930596317439230571"),
                new BigInteger("9237692551734095731335721811509563215371073110856358717074715195706992731536")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("13875495990227505228446859913301000050997840352882736374309618075415310894648"),
                new BigInteger("3497276303949216285159458753087914672946164176419127627578799181015095956873")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("18447324748162254557490058359493475348866958049718715121269546281859097875236"),
                new BigInteger("17077342491879725483408384985562872427778043781188668948568754633334966147582")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("12851056729071923779895951169999899723192332071242327892104940178549135250767"),
                new BigInteger("18634098455836127811166537054006769004625567161896081603107849451917841610029")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("17403056415026394500298873560117615010477265904720715456810417508186205303034"),
                new BigInteger("8964264574319211246675268341429720450511948605205744556261372597555758671802")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("18076699560142031249365246487867216573467696461365894405244494672668265559911"),
                new BigInteger("3846332807924276770069687030211249511052092299439390683646280741932556175212")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("77519037424879463498594143523086832364892778842456437054079427865698674768"),
                new BigInteger("6234020029278098010442369543106746611262917860119526547845899987070452927155")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("19650352048870088123355117953243496107099075237274114851765312863420178229097"),
                new BigInteger("10950108713102878453907148978492208067110447584738718802082216276647381125337")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("4089846277309905920586812937579634932927041973309126976909424779539590437521"),
                new BigInteger("16472710219322117499520866547339568760128277711833416003185110132525826992206")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("3562400560182011603204073482249013763255845752734021837260387420734067465802"),
                new BigInteger("566201039058796287201629017713992492518344077384066931553594729805860806751")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("7871742474534361706329311011956372267757125908421038101063080056794069777086"),
                new BigInteger("3388386647449561775701169844845217317795433096109859259799076496377267701513")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("7173871911007249670749514428358873088359375797528368312946388329573810755094"),
                new BigInteger("11453598252068068210646741408968788245969259663830102964256690295858306894797")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("8030036706905250057707130166764568759869289421792861235643223918662069112599"),
                new BigInteger("21710263275910884203412915512304751240888507453381604058564846537925964739440")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("10252108491481663646364239559488669473340990765161097080381759825411878346340"),
                new BigInteger("2318411219443031223556078150091452975621960559518557995101011583264341339419")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("8382798759916358141275601700827962870337446114207503865940817036384939891543"),
                new BigInteger("20195646290580584227072537779857780996174230198622241354752629301319019319830")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("3409860861889595512046375962758554547177114993541625056380996674981022078925"),
                new BigInteger("6533268803614419258954947653080079844123930052516011388860224235942385065464")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("19900371125318484880578259019112913626825499003482769033002090154409007806470"),
                new BigInteger("7111207192414851491611143534233933059865974877235228351894012799734359793110")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("21271397541842631528252310593419300180653119200089107586442372529135730988003"),
                new BigInteger("2688555989209285931424196363584054455300368074708220057620903764463581401464")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("5886576876121043358717636332812212860233972861939640917947913866135502264290"),
                new BigInteger("6326147844104426259880758446195334756846013861213437378207265716524892168506")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("13128902859394729149824157303937334996069428505107018276444662824334639894947"),
                new BigInteger("18444329305895870335316246955960144394462296257927984180751266981446675630485")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("11274098429209630128294417483088302440848007996023671818547799560579043690500"),
                new BigInteger("13626252394859635987457241891372394163218817631713354084317688164302306233628")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("21149014631849729207256892706304922131628355199958190403080613186569192403715"),
                new BigInteger("21709577982336190825597091912253255051913118938407546975241076017533144776950")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("7002655808624107464778523378781303205943576613705391082336233163141366941625"),
                new BigInteger("6066259973159651648923232247842286945028211585707448509680516714551636520523")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("12068520550543033674252359113638590577017139722512125328499423662755506689393"),
                new BigInteger("14393494617908122906623281386837266592601968918709674782379677407270057080741")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("4266451363767364819833569662913332949276293908907951959008051168195499528708"),
                new BigInteger("3096758584798287081616863539326112678635565487427292668334827991840338944149")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("21101581578823257706513744407948708459007606485197437262949088670685559188513"),
                new BigInteger("5324283950791690771607509093987776613882092884856347345472152848544886030004")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("18843153333286691505644566178010396886618592077079890128575065017465629466420"),
                new BigInteger("4674543773702741769046933785598608160155729438797894294314682517785467134355")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("12096683708924994572228328162555146811088851226831030783533896171306298725418"),
                new BigInteger("20793324826206902282780329555921182292797070505644766978149081356376061497924")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("7544557328007047733582127225601307022359787484893666085302448606181632124212"),
                new BigInteger("11756013476233046033037765003192626267548085693888580890701021937685244068698")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("18097734944168659063456376453496520468315447634785332503315087648788909672612"),
                new BigInteger("1856440567390246762480021154490641867711083663933065171343466778316296080451")
            ),
            
            new Pairing.G1Point( 
                new BigInteger("9623096166063491489103363826845355271985303939647220842793290763427957658175"),
                new BigInteger("19193198634930537658034729213867468798270912267021581331906466973246556672464")
            ),
            
        };

        return vk;
    }

    public int verify(BigInteger[] input, Proof proof) {
        BigInteger snark_scalar_field = new BigInteger("21888242871839275222246405745257275088548364400416034343698204186575808495617");
        VerifyingKey vk = verifyingKey();
        Context.require(input.length + 1 == vk.IC.length, "verifier-bad-input");
        // Compute the linear combination vk_x
        Pairing.G1Point vk_x = new Pairing.G1Point(BigInteger.ZERO,BigInteger.ZERO);
        for (int i=0; i<input.length; i++) {
            Context.require(input[i].compareTo(snark_scalar_field) < 0, "verifier-gte-snark-scalar-field");
            vk_x = Pairing.addition(vk_x, Pairing.scalar_mul(vk.IC[i+1], new Pairing.Scalar(input[0])));
        }
        vk_x = Pairing.addition(vk_x, vk.IC[0]);
        if (!Pairing.pairingProd4(Pairing.negate(proof.A), proof.B, vk.alfa1, vk.beta2, vk_x, vk.gamma2, proof.C, vk.delta2)) {
            return 1;
        }
        return 0;
    }

    @External(readonly = true)
    public boolean verifyProof(BigInteger[] a, BigInteger[][] b, BigInteger[] c, BigInteger[] input) {
        Proof proof = new Proof();
        proof.A = new Pairing.G1Point(a[0], a[1]);
        proof.B = new Pairing.G2Point(b[0][0], b[0][1], b[1][0], b[1][1]);
        proof.C = new Pairing.G1Point(c[0], c[1]);

        // TODO: Verify if copying is necessary
        return verify(input, proof) == 0;
    }
}
