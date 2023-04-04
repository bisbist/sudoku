
package io.venture23zkp.sudoku;

import java.math.BigInteger;

import score.Context;
import score.annotation.External;

public class Sudoku {
    private final Verifier verifier;

    public static int[][][] sudokuBoardList = {
            {
                    { 1, 2, 7, 5, 8, 4, 6, 9, 3 },
                    { 8, 5, 6, 3, 7, 9, 1, 2, 4 },
                    { 3, 4, 9, 6, 2, 1, 8, 7, 5 },
                    { 4, 7, 1, 9, 5, 8, 2, 3, 6 },
                    { 2, 6, 8, 7, 1, 3, 5, 4, 9 },
                    { 9, 3, 5, 4, 6, 2, 7, 1, 8 },
                    { 5, 8, 3, 2, 9, 7, 4, 6, 1 },
                    { 7, 1, 4, 8, 3, 6, 9, 5, 2 },
                    { 6, 9, 2, 1, 4, 5, 3, 0, 7 }
            },
            {
                    { 0, 2, 7, 5, 0, 4, 0, 0, 0 },
                    { 0, 0, 0, 3, 7, 0, 0, 0, 4 },
                    { 3, 0, 0, 0, 0, 0, 8, 0, 0 },
                    { 4, 7, 0, 9, 5, 8, 0, 3, 6 },
                    { 2, 6, 8, 7, 1, 0, 0, 4, 9 },
                    { 0, 0, 0, 0, 0, 2, 0, 1, 8 },
                    { 0, 8, 3, 0, 9, 0, 4, 0, 0 },
                    { 7, 1, 0, 0, 0, 0, 9, 0, 2 },
                    { 0, 0, 0, 0, 0, 5, 0, 0, 7 }
            },
            {
                    { 0, 0, 0, 0, 0, 6, 0, 0, 0 },
                    { 0, 0, 7, 2, 0, 0, 8, 0, 0 },
                    { 9, 0, 6, 8, 0, 0, 0, 1, 0 },
                    { 3, 0, 0, 7, 0, 0, 0, 2, 9 },
                    { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                    { 4, 0, 0, 5, 0, 0, 0, 7, 0 },
                    { 6, 5, 0, 1, 0, 0, 0, 0, 0 },
                    { 8, 0, 1, 0, 5, 0, 3, 0, 0 },
                    { 7, 9, 2, 0, 0, 0, 0, 0, 4 }
            }
    };

    public Sudoku() {
        this.verifier = new Verifier();
    }

    @External(readonly = true)
    public boolean verifyProof(BigInteger[] a, BigInteger[][] b, BigInteger[] c, BigInteger[] input) {
        return this.verifier.verifyProof(a, b, c, input);
    }

    private boolean verifySudokuBoard(BigInteger[] input) {
        boolean isEqual = true;
        for (int i = 0; i < sudokuBoardList.length; ++i) {
            isEqual = true;
            for (int j = 0; j < sudokuBoardList[i].length; ++j) {
                for (int k = 0; k < sudokuBoardList[i][j].length; ++k) {
                    if (BigInteger.valueOf(sudokuBoardList[i][j][k]).equals(input[9 * j + k])) {
                        isEqual = false;
                        break;
                    }
                }
            }
            if (isEqual == true) {
                return isEqual;
            }
        }
        return isEqual;
    }

    @External(readonly = true)
    public boolean verifySudoku(BigInteger[] a, BigInteger[][] b, BigInteger[] c, BigInteger[] input) {
        if (!verifySudokuBoard(input)) {
            throw new RuntimeException("This board does not exist");
        }
        if (!verifyProof(a, b, c, input)) {
            throw new RuntimeException("Failed proof check");
        }
        return true;
    }

    private int getRandomInt(byte[] seed, int mod) {
        // ByteBuffer data = ByteBuffer.allocate(64);
        // data.putLong(0, Context.getBlockHeight());
        // data.putLong(8, Context.getBlockTimestamp());
        // data.put(Context.getCaller().toByteArray(), 16, 20);
        // data.put(seed, 36, 8);

        // BigInteger rand = new BigInteger(1, Context.hash("sha-256", data.array()));
        // return rand.mod(BigInteger.valueOf(mod)).intValue();
        return (int) (Context.getBlockTimestamp() % mod); // note: this is not a true random number but enough for our case
    }

    private BigInteger[][] pickRandomBoard(String stringTime) {

        int randPosition = getRandomInt(stringTime.getBytes(), sudokuBoardList.length);
        int[][] sb = sudokuBoardList[randPosition];

        BigInteger[][] sudokuBoard = new BigInteger[][] {};
        for (int i = 0; i < sb.length; i++) {
            for (int j = 0; j < sb[i].length; j++) {
                sudokuBoard[i][j] = BigInteger.valueOf(sb[i][j]);
            }
        }

        return sudokuBoard;
    }

    @External(readonly = true)
    public BigInteger[][] generateSudokuBoard(String stringTime) {
        return pickRandomBoard(stringTime);
    }
}
